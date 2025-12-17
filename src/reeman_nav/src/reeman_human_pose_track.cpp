#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/char.hpp>
#include "reeman_api_interface/srv/post_speed.hpp"
#include "custom_interface/srv/logistic_nav_status.hpp"

#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class humanTrackNode : public rclcpp::Node
{
    public:
    humanTrackNode() : Node("human_tracking")
    {
        auto callback_sub = [this](std_msgs::msg::Int32::UniquePtr msg)->void
        {
            humanState = msg->data;
        };
        subscriber_ = this->create_subscription<std_msgs::msg::Int32>("/vision/human_zone", 10, callback_sub);
        auto callback = [this]()->void
        {
            char logisticNavStatus;
            std::shared_ptr<rclcpp::Node> node_logisticNavStatus = rclcpp::Node::make_shared("logistic_nav_status_human_tracker_client");
            rclcpp::Client<custom_interface::srv::LogisticNavStatus>::SharedPtr client_logisticNavStatus_humanTracking = node_logisticNavStatus->create_client<custom_interface::srv::LogisticNavStatus>("logistic_nav_status");
            auto req_logisticNavStatus = std::make_shared<custom_interface::srv::LogisticNavStatus::Request>();
            req_logisticNavStatus->set__action(0);
            req_logisticNavStatus->set__state(0);
            while (!client_logisticNavStatus_humanTracking->wait_for_service(100ms))
            {
                if (!rclcpp::ok()) 
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the logistic_nav_status service. Exiting.");
                    return;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "logistic_nav_status service not available, waiting again...");
            }
            auto res_logisticNavStatus = client_logisticNavStatus_humanTracking->async_send_request(req_logisticNavStatus);
            if(rclcpp::spin_until_future_complete(node_logisticNavStatus, res_logisticNavStatus) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto getRes_logisticNavState = res_logisticNavStatus.get();
                logisticNavStatus = getRes_logisticNavState->status;
            } 
            else 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service logistic_nav_status");
            }

            if(logisticNavStatus != 0)
            {
                return;
            }

            std::shared_ptr<rclcpp::Node> node_postReemanSpeed_humanTracking = rclcpp::Node::make_shared("post_reeman_speed_human_tracking_client");
            rclcpp::Client<reeman_api_interface::srv::PostSpeed>::SharedPtr client_postReemanSpeed_humanTracking = node_postReemanSpeed_humanTracking->create_client<reeman_api_interface::srv::PostSpeed>("post_reeman_speed");
            auto req_postSpeed = std::make_shared<reeman_api_interface::srv::PostSpeed::Request>();
            req_postSpeed->set__vx(0);
            if(humanState == 0)
            {
                req_postSpeed->set__vth(0.5);
            }
            else if(humanState == 3)
            {
                req_postSpeed->set__vth(-0.5);
            }
            while (!client_postReemanSpeed_humanTracking->wait_for_service(100ms))
            {
                if (!rclcpp::ok()) 
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the post_reeman_speed service. Exiting.");
                    return;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "post_reeman_speed service not available, waiting again...");
            }
            auto res_postSpeed = client_postReemanSpeed_humanTracking->async_send_request(req_postSpeed);
            if(rclcpp::spin_until_future_complete(node_postReemanSpeed_humanTracking, res_postSpeed) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto getRes_postSpeed = res_postSpeed.get();
                std::string status = getRes_postSpeed->status;
            } 
            else 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service post_reeman_speed");
            }
        };
        timer_ = this->create_wall_timer(300ms, callback);
    }
    private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    int humanState;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<humanTrackNode>());
    rclcpp::shutdown();
    return 0;
}