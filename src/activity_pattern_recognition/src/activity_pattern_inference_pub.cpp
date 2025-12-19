#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <vector>
#include <string>
#include <cmath>

#include <tensorflow/c/c_api.h>

#include "custom_interface/msg/dual_leg.hpp"
#include "custom_interface/msg/activity_forecast.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "activity_pattern_recognition/scaler.h"          // exported mean/std
#include "activity_pattern_recognition/scaler_utils.h"    // scale_input()
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>


// ==========================================================
// CONFIG
// ==========================================================

constexpr int WINDOW_SIZE = 10;
constexpr int FORECAST_STEPS = 4;
constexpr int FEATURE_DIM = 6;

// ==========================================================
// ACTION MAP
// ==========================================================

static const char* ACTION_MAP[] = {
    "sitting",
    "standing",
    "walking",
    "pre-standing",
    "pre-sitting"
};

// ==========================================================
// TensorFlow helpers
// ==========================================================

#define TF_CHECK(status) \
  if (TF_GetCode(status) != TF_OK) { \
    throw std::runtime_error(TF_Message(status)); \
  }

// --------------------------------------------------
// TensorFlow: load .pb into TF_Buffer
// --------------------------------------------------
TF_Buffer* ReadBufferFromFile(const char* file)
{
    std::ifstream f(file, std::ios::binary | std::ios::ate);
    if (!f.is_open())
        return nullptr;

    std::streamsize size = f.tellg();
    f.seekg(0, std::ios::beg);

    char* data = static_cast<char*>(malloc(size));
    if (!f.read(data, size)) {
        free(data);
        return nullptr;
    }

    TF_Buffer* buf = TF_NewBuffer();
    buf->data = data;
    buf->length = size;
    buf->data_deallocator = [](void* data, size_t) {
        free(data);
    };

    return buf;
}

// ==========================================================
// NODE
// ==========================================================

class LSTMInferenceNode : public rclcpp::Node
{
public:
    LSTMInferenceNode()
    : Node("lstm_inference_node")
    {
        // --------------------------------------------------
        // Initialize state
        // --------------------------------------------------
        left_leg_angle_ = 0.0f;
        right_leg_angle_ = 0.0f;
        x_abs_ = y_abs_ = 0.0f;
        x_prev_ = y_prev_ = 0.0f;

        // --------------------------------------------------
        // TensorFlow load
        // --------------------------------------------------
        load_model();
        // --------------------------------------------------
        // ROS interfaces
        // --------------------------------------------------
        sub_dual_leg_ = create_subscription<custom_interface::msg::DualLeg>(
            "dual_leg", 10,
            std::bind(&LSTMInferenceNode::dual_leg_cb, this, std::placeholders::_1)
        );

        sub_uwb_ = create_subscription<geometry_msgs::msg::Pose2D>(
            "uwb_pose2d", 10,
            std::bind(&LSTMInferenceNode::uwb_cb, this, std::placeholders::_1)
        );

        pub_ = create_publisher<custom_interface::msg::ActivityForecast>(
            "activity_forecast", 10
        );

        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&LSTMInferenceNode::inference_cb, this)
        );
    }

    ~LSTMInferenceNode()
    {
        TF_DeleteSession(session_, status_);
        TF_DeleteGraph(graph_);
        TF_DeleteStatus(status_);
    }

private:
    // --------------------------------------------------
    // ROS callbacks
    // --------------------------------------------------

    void dual_leg_cb(const custom_interface::msg::DualLeg::SharedPtr msg)
    {
        left_leg_angle_  = msg->left_angle;
        right_leg_angle_ = msg->right_angle;
    }

    void uwb_cb(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        auto now = this->now();

        if (last_uwb_time_.nanoseconds() > 0) {
            double dt = (now - last_uwb_time_).seconds();

            acc_dx_ += std::abs(msg->x - x_abs_);
            acc_dy_ += std::abs(msg->y - y_abs_);
        }

        x_abs_ = msg->x;
        y_abs_ = msg->y;
        last_uwb_time_ = now;
        /*
        x_abs_ = msg->x;
        y_abs_ = msg->y;

        x_rel_ = std::abs(x_abs_ - x_prev_);
        y_rel_ = std::abs(y_abs_ - y_prev_);

        x_prev_ = x_abs_;
        y_prev_ = y_abs_;
        */
    }

    // --------------------------------------------------
    // Inference
    // --------------------------------------------------

    void inference_cb()
    {
        /*
        float feature[FEATURE_DIM] = {
            left_leg_angle_,
            right_leg_angle_,
            x_rel_,
            y_rel_,
            x_abs_,
            y_abs_
        };
        */
       float feature[FEATURE_DIM] = {
        left_leg_angle_,
        right_leg_angle_,
        static_cast<float>(acc_dx_),
        static_cast<float>(acc_dy_),
        x_abs_,
        y_abs_
        };

        acc_dx_ = 0.0;
        acc_dy_ = 0.0;

        // scale per timestep
        scale_input(feature, FEATURE_DIM,
                    input_scaler_mean,
                    input_scaler_std);

        buffer_.push_back(std::vector<float>(feature, feature + FEATURE_DIM));
        if (buffer_.size() > WINDOW_SIZE)
            buffer_.pop_front();

        if (buffer_.size() < WINDOW_SIZE)
            return;

        // Prepare input tensor
        std::vector<float> input_data(WINDOW_SIZE * FEATURE_DIM);
        for (int t = 0; t < WINDOW_SIZE; ++t)
            memcpy(&input_data[t * FEATURE_DIM],
                   buffer_[t].data(),
                   FEATURE_DIM * sizeof(float));

        int64_t dims[3] = {1, WINDOW_SIZE, FEATURE_DIM};

        TF_Tensor* input_tensor = TF_AllocateTensor(
            TF_FLOAT, dims, 3,
            input_data.size() * sizeof(float)
        );

        std::memcpy(
            TF_TensorData(input_tensor),
            input_data.data(),
            input_data.size() * sizeof(float)
        );


        TF_Output input = {input_op_, 0};

        std::vector<TF_Tensor*> outputs(output_ops_.size(), nullptr);
        

        TF_SessionRun(
            session_,
            nullptr,
            &input,
            &input_tensor,
            1,
            output_ops_.data(),
            outputs.data(),
            outputs.size(),
            nullptr, 0, nullptr,
            status_
        );
        TF_CHECK(status_);

        // --------------------------------------------------
        // Publish
        // --------------------------------------------------
        custom_interface::msg::ActivityForecast msg;

        // action forecasts
        for (int i = 0; i < FORECAST_STEPS; ++i)
        {
            float* probs = static_cast<float*>(TF_TensorData(outputs[i]));
            int best = 0;
            for (int j = 1; j < 5; ++j)
                if (probs[j] > probs[best]) best = j;

            msg.actions.push_back(ACTION_MAP[best]);
            msg.confidences.push_back(probs[best]);
        }

        // ETA (last output)
        float* eta = static_cast<float*>(TF_TensorData(outputs.back()));
        msg.time_to_arrival = eta[0];

        pub_->publish(msg);

        // cleanup
        TF_DeleteTensor(input_tensor);
        for (auto* t : outputs) TF_DeleteTensor(t);
    }

    // --------------------------------------------------
    // TensorFlow loading
    // --------------------------------------------------

    void load_model()
{
    status_ = TF_NewStatus();
    graph_  = TF_NewGraph();

    // --------------------------------------------------
    // Read frozen graph
    // --------------------------------------------------
    std::string model_path =
        ament_index_cpp::get_package_share_directory("activity_pattern_recognition") +
        "/model/model_frozen.pb";

    TF_Buffer* graph_def = ReadBufferFromFile(model_path.c_str());
    if (!graph_def) {
        throw std::runtime_error("Failed to load model_frozen.pb");
    }

    TF_ImportGraphDefOptions* import_opts = TF_NewImportGraphDefOptions();
    TF_GraphImportGraphDef(graph_, graph_def, import_opts, status_);
    TF_CHECK(status_);

    TF_DeleteImportGraphDefOptions(import_opts);
    TF_DeleteBuffer(graph_def);

    // --------------------------------------------------
    // Create session
    // --------------------------------------------------
    TF_SessionOptions* sess_opts = TF_NewSessionOptions();
    session_ = TF_NewSession(graph_, sess_opts, status_);
    TF_CHECK(status_);

    // DEBUG: dump all ops
/*
size_t pos = 0;
TF_Operation* op = nullptr;

std::cout << "=== TensorFlow Graph Ops ===" << std::endl;
while ((op = TF_GraphNextOperation(graph_, &pos)) != nullptr)
{
    std::cout << TF_OperationName(op) << std::endl;
}
std::cout << "===========================" << std::endl;
*/


    // --------------------------------------------------
    // Resolve input / output ops
    // --------------------------------------------------
    input_op_ = TF_GraphOperationByName(graph_, "x");
    if (!input_op_) throw std::runtime_error("Input op not found");
    output_ops_ = {
    {TF_GraphOperationByName(graph_, "Identity"), 0},
    {TF_GraphOperationByName(graph_, "Identity_1"), 0},
    {TF_GraphOperationByName(graph_, "Identity_2"), 0},
    {TF_GraphOperationByName(graph_, "Identity_3"), 0},
    {TF_GraphOperationByName(graph_, "Identity_4"), 0}
    };
    for (auto& o : output_ops_)
        if (!o.oper) throw std::runtime_error("Output op not found");
    
}


    // --------------------------------------------------
    // Members
    // --------------------------------------------------
    double acc_dx_ = 0.0;
    double acc_dy_ = 0.0;
    rclcpp::Time last_uwb_time_;
    float left_leg_angle_, right_leg_angle_;
    float x_abs_, y_abs_, x_prev_, y_prev_, x_rel_, y_rel_;

    std::deque<std::vector<float>> buffer_;

    rclcpp::Subscription<custom_interface::msg::DualLeg>::SharedPtr sub_dual_leg_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_uwb_;
    rclcpp::Publisher<custom_interface::msg::ActivityForecast>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // TensorFlow
    TF_Status* status_;
    TF_Graph* graph_;
    TF_Session* session_;
    TF_Operation* input_op_;
    std::vector<TF_Output> output_ops_;
};

// ==========================================================
// MAIN
// ==========================================================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LSTMInferenceNode>());
    rclcpp::shutdown();
    return 0;
}
