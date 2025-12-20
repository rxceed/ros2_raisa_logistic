#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <chrono>
#include <cstring>
#include <cmath>
#include <fstream>
#include <vector>
#include <filesystem>

#include "tensorflow/c/c_api.h"
#include "activity_pattern_recognition/scaler.h" 

#include "geometry_msgs/msg/pose2_d.hpp"
#include "custom_interface/msg/dual_leg.hpp"
#include "custom_interface/msg/activity_forecast.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

static constexpr int FEATURE_DIM = 5; 
static constexpr int WINDOW_SIZE = 10;
static constexpr int PRED_HORIZON = 4;
static constexpr int NUM_CLASSES = 6;

static void NoOpDeallocator(void*, size_t, void*) {}

class LSTMNode : public rclcpp::Node {
public:
    LSTMNode() : Node("activity_pattern_forecast") {
        // 1. Resolve Path
        std::string package_share = ament_index_cpp::get_package_share_directory("activity_pattern_recognition");
        std::string model_path = package_share + "/model/model.pb";
        
        RCLCPP_INFO(this->get_logger(), "Loading Model from: %s", model_path.c_str());

        // 2. Load and Inspect
        try {
            load_model(model_path);
            dump_graph();
            find_io_nodes(); // Automatically detect names if defaults fail
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Initialization failed: %s", e.what());
            return;
        }

        // 3. ROS Interfaces
        pose_sub_ = create_subscription<geometry_msgs::msg::Pose2D>(
            "uwb_pose2d", 10, std::bind(&LSTMNode::pose_callback, this, std::placeholders::_1));

        angle_sub_ = create_subscription<custom_interface::msg::DualLeg>(
            "dual_leg", 10, std::bind(&LSTMNode::angle_callback, this, std::placeholders::_1));

        pub_ = create_publisher<custom_interface::msg::ActivityForecast>("action_forecast", 10);

        timer_ = create_wall_timer(1s, std::bind(&LSTMNode::run_inference, this));
        
        RCLCPP_INFO(this->get_logger(), "Node Ready.");
    }

    ~LSTMNode() {
        if (session_) TF_DeleteSession(session_, status_);
        if (graph_) TF_DeleteGraph(graph_);
        if (status_) TF_DeleteStatus(status_);
    }

private:
    TF_Graph* graph_ = nullptr;
    TF_Session* session_ = nullptr;
    TF_Status* status_ = nullptr;
    
    // Detected node names
    std::string input_node_name_ = "input_1";
    std::string output_node_name_ = "Identity";

    float current_x_ = 0.0f, current_y_ = 0.0f;
    float left_angle_deg_ = 0.0f, right_angle_deg_ = 0.0f;
    float time_working_counter_ = 0.0f;
    bool pose_ready_ = false, angle_ready_ = false;

    struct Sample { float features[FEATURE_DIM]; };
    std::deque<Sample> window_;

    void load_model(const std::string& path) {
        if (!std::filesystem::exists(path)) {
            throw std::runtime_error("File not found: " + path);
        }

        graph_ = TF_NewGraph();
        status_ = TF_NewStatus();
        
        TF_Buffer* graph_def = read_pb(path);
        if (!graph_def) throw std::runtime_error("Failed to read .pb buffer");

        TF_ImportGraphDefOptions* import_opts = TF_NewImportGraphDefOptions();
        TF_GraphImportGraphDef(graph_, graph_def, import_opts, status_);
        
        TF_DeleteImportGraphDefOptions(import_opts);
        TF_DeleteBuffer(graph_def);

        if (TF_GetCode(status_) != TF_OK) {
            throw std::runtime_error("Graph import failed: " + std::string(TF_Message(status_)));
        }

        TF_SessionOptions* sess_opts = TF_NewSessionOptions();
        session_ = TF_NewSession(graph_, sess_opts, status_);
        TF_DeleteSessionOptions(sess_opts);

        if (TF_GetCode(status_) != TF_OK) {
            throw std::runtime_error("Session creation failed: " + std::string(TF_Message(status_)));
        }
    }

    void dump_graph() {
        size_t pos = 0;
        TF_Operation* oper;
        RCLCPP_INFO(this->get_logger(), "--- START GRAPH DUMP ---");
        while ((oper = TF_GraphNextOperation(graph_, &pos)) != nullptr) {
            printf("Node: %s (%s)\n", TF_OperationName(oper), TF_OperationOpType(oper));
        }
        RCLCPP_INFO(this->get_logger(), "--- END GRAPH DUMP ---");
    }

    void find_io_nodes() {
        // Fuzzy search for input/output if defaults aren't found
        if (!TF_GraphOperationByName(graph_, input_node_name_.c_str())) {
            size_t pos = 0; TF_Operation* oper;
            while ((oper = TF_GraphNextOperation(graph_, &pos)) != nullptr) {
                if (std::string(TF_OperationOpType(oper)) == "Placeholder") {
                    input_node_name_ = TF_OperationName(oper);
                    break;
                }
            }
        }
        if (!TF_GraphOperationByName(graph_, output_node_name_.c_str())) {
            size_t pos = 0; TF_Operation* oper;
            while ((oper = TF_GraphNextOperation(graph_, &pos)) != nullptr) {
                if (std::string(TF_OperationOpType(oper)) == "Identity") {
                    output_node_name_ = TF_OperationName(oper); // Takes last identity found
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Using Input: %s, Output: %s", 
                    input_node_name_.c_str(), output_node_name_.c_str());
    }

    void run_inference() {
        if (!pose_ready_ || !angle_ready_) return;

        // 1s Working Logic
        if ((left_angle_deg_ < 10.0f || left_angle_deg_ > 350.0f) && 
            (right_angle_deg_ < 10.0f || right_angle_deg_ > 350.0f)) {
            time_working_counter_ += 1.0f;
        } else {
            time_working_counter_ = 0.0f;
        }

        Sample s;
        s.features[0] = current_x_;
        s.features[1] = current_y_;
        s.features[2] = time_working_counter_;
        s.features[3] = std::cos(left_angle_deg_ * 0.0174533f);
        s.features[4] = std::cos(right_angle_deg_ * 0.0174533f);

        scale_sample(s);
        window_.push_back(s);
        if (window_.size() > WINDOW_SIZE) window_.pop_front();
        if (window_.size() < WINDOW_SIZE) return;

        // Inference
        std::vector<float> input_flat;
        for (const auto& samp : window_) 
            for (int i = 0; i < FEATURE_DIM; ++i) input_flat.push_back(samp.features[i]);

        int64_t dims[3] = {1, WINDOW_SIZE, FEATURE_DIM};
        TF_Tensor* input_tensor = TF_NewTensor(TF_FLOAT, dims, 3, input_flat.data(), 
                                               sizeof(float) * input_flat.size(), &NoOpDeallocator, nullptr);

        TF_Output input_op = {TF_GraphOperationByName(graph_, input_node_name_.c_str()), 0};
        TF_Output output_op = {TF_GraphOperationByName(graph_, output_node_name_.c_str()), 0};
        TF_Tensor* output_tensor = nullptr;

        TF_SessionRun(session_, nullptr, &input_op, &input_tensor, 1, &output_op, &output_tensor, 1, nullptr, 0, nullptr, status_);

        if (TF_GetCode(status_) == TF_OK && output_tensor) {
            publish_result(output_tensor);
            TF_DeleteTensor(output_tensor);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Run failed: %s", TF_Message(status_));
        }
        TF_DeleteTensor(input_tensor);
    }

    void publish_result(TF_Tensor* tensor) {
        float* data = static_cast<float*>(TF_TensorData(tensor));
        custom_interface::msg::ActivityForecast msg;
        int preds[4];
        for (int h = 0; h < PRED_HORIZON; ++h) {
            int best = 0; float max_v = -1e9;
            for (int c = 0; c < NUM_CLASSES; ++c) {
                float v = data[h * NUM_CLASSES + c];
                if (v > max_v) { max_v = v; best = c; }
            }
            preds[h] = best;
        }
        msg.t_plus_1 = preds[0]; msg.t_plus_2 = preds[1];
        msg.t_plus_3 = preds[2]; msg.t_plus_4 = preds[3];
        pub_->publish(msg);
    }

    void scale_sample(Sample& s) {
        for (int i = 0; i < FEATURE_DIM; ++i) 
            s.features[i] = s.features[i] * SCALER_SCALE[i] + SCALER_MIN[i];
    }

    TF_Buffer* read_pb(const std::string& path) {
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file.is_open()) return nullptr;
        std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);
        void* data = malloc(size);
        if (!file.read(static_cast<char*>(data), size)) { free(data); return nullptr; }
        TF_Buffer* buf = TF_NewBuffer();
        buf->data = data; buf->length = size;
        buf->data_deallocator = [](void* d, size_t) { free(d); };
        return buf;
    }

    void pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg) { current_x_ = msg->x; current_y_ = msg->y; pose_ready_ = true; }
    void angle_callback(const custom_interface::msg::DualLeg::SharedPtr msg) { left_angle_deg_ = msg->left_angle; right_angle_deg_ = msg->right_angle; angle_ready_ = true; }

    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
    rclcpp::Subscription<custom_interface::msg::DualLeg>::SharedPtr angle_sub_;
    rclcpp::Publisher<custom_interface::msg::ActivityForecast>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LSTMNode>());
    rclcpp::shutdown();
    return 0;
}