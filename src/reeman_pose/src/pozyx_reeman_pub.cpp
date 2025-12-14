#include <iostream>
#include <memory>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include <nlohmann/json.hpp>
#include "reeman_api_interface/srv/get_pose.hpp"
#include "custom_interface/srv/pozyx_to_reeman.hpp"

using namespace std::chrono_literals;

class pozyxReemanPublisher : public rclcpp::Node
{
    public:
    pozyxReemanPublisher() : Node("pozyx_reeman_publisher")
    {
        auto callback_sub = [this](geometry_msgs::msg::Pose2D::UniquePtr msg)->void 
        {
            p_x = msg->x;
            p_y = msg->y;
            p_theta = msg->theta;
        };
        subscriber_ = this->create_subscription<geometry_msgs::msg::Pose2D>("uwb_pose2d", 10, callback_sub);
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("pozyx_reeman", 10);
        auto callback = [this]()->void {
            auto message = geometry_msgs::msg::Pose2D();
            std::shared_ptr<rclcpp::Node> node_getReemanPose = rclcpp::Node::make_shared("pozyx_to_reeman_client");
            rclcpp::Client<custom_interface::srv::PozyxToReeman>::SharedPtr client_getReemanPose = node_getReemanPose->create_client<custom_interface::srv::PozyxToReeman>("pozyx_to_reeman");
            auto req = std::make_shared<custom_interface::srv::PozyxToReeman::Request>();
            req->set__pozyx_x(p_x);
            req->set__pozyx_y(p_y);
            while (!client_getReemanPose->wait_for_service(100ms))
            {
                if (!rclcpp::ok()) 
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }
            auto res = client_getReemanPose->async_send_request(req);
            if(rclcpp::spin_until_future_complete(node_getReemanPose, res) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto getRes = res.get();
                message.set__x(getRes->reeman_x);
                message.set__y(getRes->reeman_y);
                message.set__theta(0);
            } 
            else 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_reeman_pose");
            }
            this->publisher_->publish(message);
        };
        timer_ = this->create_wall_timer(300ms, callback);
    }

    private:
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    float p_x = 0, p_y = 0, p_theta = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pozyxReemanPublisher>());
    rclcpp::shutdown();
    return 0;
}