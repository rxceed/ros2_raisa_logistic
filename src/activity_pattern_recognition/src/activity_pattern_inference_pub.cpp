#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <chrono>
#include <cstring>

#include "tensorflow/c/c_api.h"
#include "activity_pattern_recognition/scaler.h"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "custom_interface/msg/dual_leg.hpp"
#include "custom_interface/msg/activity_forecast.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>



using namespace std::chrono_literals;

static constexpr int WINDOW_SIZE = 10;
static constexpr int PRED_HORIZON = 4;
static constexpr int NUM_CLASSES = 7;

static void NoOpDeallocator(void*, size_t, void*) {}

std::string model_path =
    ament_index_cpp::get_package_share_directory("activity_pattern_recognition") +
    "/model/bilstm_action_predictor.pb";



class LSTMNode : public rclcpp::Node {
public:
    LSTMNode() : Node("activity_pattern_forecast") {
        load_model();
        dump_graph();
        pose_sub_ = create_subscription<geometry_msgs::msg::Pose2D>(
            "uwb_pose2d", 10,
            std::bind(&LSTMNode::pose_callback, this, std::placeholders::_1)
        );

        angle_sub_ = create_subscription<custom_interface::msg::DualLeg>(
            "dual_leg", 10,
            std::bind(&LSTMNode::angle_callback, this, std::placeholders::_1)
        );

        pub_ = create_publisher<custom_interface::msg::ActivityForecast>(
            "action_forecast", 10
        );

        timer_ = create_wall_timer(
            1s,
            std::bind(&LSTMNode::run_inference, this)
        );
    }

    ~LSTMNode() {
        TF_DeleteSession(session_, status_);
        TF_DeleteGraph(graph_);
        TF_DeleteStatus(status_);
    }

private:
    /* ================= TF ================= */

    TF_Graph* graph_;
    TF_Session* session_;
    TF_Status* status_;

    void load_model() {
        graph_ = TF_NewGraph();
        status_ = TF_NewStatus();

        TF_SessionOptions* opts = TF_NewSessionOptions();
        session_ = TF_NewSession(graph_, opts, status_);
        TF_DeleteSessionOptions(opts);

        TF_Buffer* graph_def = read_pb(model_path);
        TF_ImportGraphDefOptions* import_opts = TF_NewImportGraphDefOptions();
        TF_GraphImportGraphDef(graph_, graph_def, import_opts, status_);

        TF_DeleteImportGraphDefOptions(import_opts);
        TF_DeleteBuffer(graph_def);

        if (TF_GetCode(status_) != TF_OK) {
            throw std::runtime_error(TF_Message(status_));
        }
    }

    /* ================= Data ================= */

    struct Sample {
        float features[FEATURE_DIM];
    };

    std::deque<Sample> window_;
    Sample latest_sample_;

    void pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        latest_sample_.features[0] = msg->x;
        latest_sample_.features[1] = msg->y;
        pose_ready_ = true;
    }

    void angle_callback(const custom_interface::msg::DualLeg::SharedPtr msg) {
        latest_sample_.features[2] = std::cos(msg->left_angle);
        latest_sample_.features[3] = std::cos(msg->right_angle);
        angle_ready_ = true;
    }

    /* ================= Inference ================= */


void run_inference()
{
    if (!pose_ready_ || !angle_ready_)
        return;

    scale_sample(latest_sample_);
    window_.push_back(latest_sample_);

    if (window_.size() < WINDOW_SIZE)
        return;

    if (window_.size() > WINDOW_SIZE)
        window_.pop_front();

    input_buffer_.resize(WINDOW_SIZE * FEATURE_DIM);

    for (int t = 0; t < WINDOW_SIZE; ++t)
        for (int f = 0; f < FEATURE_DIM; ++f)
            input_buffer_[t * FEATURE_DIM + f] =
                window_[t].features[f];

    static constexpr int64_t INPUT_DIMS[3] = {
        1, WINDOW_SIZE, FEATURE_DIM
    };

    TF_Tensor* input_tensor = TF_NewTensor(
        TF_FLOAT,
        INPUT_DIMS,
        3,
        input_buffer_.data(),
        sizeof(float) * input_buffer_.size(),
        &NoOpDeallocator,
        nullptr
    );

    TF_Operation* input_oper =
        TF_GraphOperationByName(graph_, "input");
    TF_Operation* output_oper =
        TF_GraphOperationByName(graph_, "Identity");

    if (!input_oper || !output_oper)
    {
        RCLCPP_FATAL(get_logger(), "Graph ops not found");
        TF_DeleteTensor(input_tensor);
        return;
    }

    TF_Output input_op{input_oper, 0};
    TF_Output output_op{output_oper, 0};

    TF_Tensor* output_tensor = nullptr;

    TF_SessionRun(
        session_,
        nullptr,
        &input_op, &input_tensor, 1,
        &output_op, &output_tensor, 1,
        nullptr, 0,
        nullptr,
        status_
    );

    TF_DeleteTensor(input_tensor);

    if (TF_GetCode(status_) != TF_OK || !output_tensor)
    {
        RCLCPP_ERROR(get_logger(), "TF_Run failed: %s",
                     TF_Message(status_));
        return;
    }

    publish_result(output_tensor);
    TF_DeleteTensor(output_tensor);
}


    void publish_result(TF_Tensor* tensor) {
        auto* data = static_cast<float*>(TF_TensorData(tensor));

        custom_interface::msg::ActivityForecast msg;
        for (int h = 0; h < PRED_HORIZON; ++h) {
            int best = 0;
            float maxv = data[h * NUM_CLASSES];
            for (int c = 1; c < NUM_CLASSES; ++c) {
                float v = data[h * NUM_CLASSES + c];
                if (v > maxv) {
                    maxv = v;
                    best = c;
                }
            }
            (&msg.t_plus_1)[h] = best;
        }

        pub_->publish(msg);
    }

    /* ================= Utils ================= */

   void scale_sample(Sample& s)
    {
        for (int i = 0; i < FEATURE_DIM; ++i)
        {
            s.features[i] = s.features[i] * SCALER_SCALE[i] + SCALER_MIN[i];
        }
    }


TF_Buffer* read_pb(const std::string& path)
{
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) {
        RCLCPP_ERROR(get_logger(), "Failed to open model file: %s", path.c_str());
        return nullptr;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    rewind(f);

    if (size <= 0) {
        fclose(f);
        RCLCPP_ERROR(get_logger(), "Model file is empty");
        return nullptr;
    }

    void* data = malloc(size);
    if (!data) {
        fclose(f);
        RCLCPP_ERROR(get_logger(), "Failed to allocate memory for model");
        return nullptr;
    }

    fread(data, 1, size, f);
    fclose(f);

    TF_Buffer* buf = TF_NewBuffer();
    buf->data = data;
    buf->length = size;
    buf->data_deallocator = [](void* d, size_t) { free(d); };

    return buf;
}


    void dump_graph()
    {
    
        size_t pos = 0;
        TF_Operation* op = nullptr;

        std::cout << "=== TensorFlow Graph Ops ===" << std::endl;
        while ((op = TF_GraphNextOperation(graph_, &pos)) != nullptr)
        {
            std::cout << TF_OperationName(op) << std::endl;
        }
        std::cout << "===========================" << std::endl;


    }


    /* ================= ROS ================= */

    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
    rclcpp::Subscription<custom_interface::msg::DualLeg>::SharedPtr angle_sub_;
    rclcpp::Publisher<custom_interface::msg::ActivityForecast>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<float> input_buffer_;
    bool pose_ready_ = false;
    bool angle_ready_ = false;


};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LSTMNode>());
    rclcpp::shutdown();
    return 0;
}
