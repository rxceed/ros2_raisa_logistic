#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include "reeman_api_interface/srv/post_speed.hpp"

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
            std::shared_ptr<rclcpp::Node> node_postReemanSpeed_humanTracking = rclcpp::Node::make_shared("post_reeman_speed_human_tracking_client");
            rclcpp::Client<reeman_api_interface::srv::PostSpeed>::SharedPtr client_postReemanSpeed_humanTracking = node_postReemanSpeed_humanTracking->create_client<reeman_api_interface::srv::PostSpeed>("post_reeman_speed");
            auto req = std::make_shared<reeman_api_interface::srv::PostSpeed::Request>();
            req->set__vx(0);
            if(humanState == 0)
            {
                req->set__vth(0.5);
            }
            else if(humanState == 3)
            {
                req->set__vth(-0.5);
            }
            while (!client_postReemanSpeed_humanTracking->wait_for_service(100ms))
            {
                if (!rclcpp::ok()) 
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }
            auto res = client_postReemanSpeed_humanTracking->async_send_request(req);
            if(rclcpp::spin_until_future_complete(node_postReemanSpeed_humanTracking, res) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto getRes = res.get();
                std::string status = getRes->status;
            } 
            else 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_reeman_pose");
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