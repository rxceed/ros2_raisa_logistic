#include <iostream>
#include <memory>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include <nlohmann/json.hpp>
#include "reeman_api_interface/srv/get_pose.hpp"
#include "reeman_api_interface/srv/get_speed.hpp"
#include "reeman_api_interface/msg/reeman_speed.hpp"

using namespace std::chrono_literals;

class publisher : public rclcpp::Node
{
    public:
    publisher() : Node("pose_speed_publisher")
    {
        posePublisher = this->create_publisher<geometry_msgs::msg::Pose2D>("reeman_pose2d", 10);
        speedPublisher = this->create_publisher<reeman_api_interface::msg::ReemanSpeed>("reeman_speed", 10);
        auto callback_pose = [this]()->void 
        {
            auto message_pose = geometry_msgs::msg::Pose2D();
            std::shared_ptr<rclcpp::Node> node_getReemanPose = rclcpp::Node::make_shared("get_reeman_pose_client");
            rclcpp::Client<reeman_api_interface::srv::GetPose>::SharedPtr client_getReemanPose = node_getReemanPose->create_client<reeman_api_interface::srv::GetPose>("get_reeman_pose");
            auto req_pose = std::make_shared<reeman_api_interface::srv::GetPose::Request>();
            //Get pose
            while(!client_getReemanPose->wait_for_service(100ms)) 
            {
                if (!rclcpp::ok()) 
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }
            auto res_pose = client_getReemanPose->async_send_request(req_pose);
            if(rclcpp::spin_until_future_complete(node_getReemanPose, res_pose) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto getResPose = res_pose.get();
                message_pose.set__x(getResPose->x);
                message_pose.set__y(getResPose->y);
                message_pose.set__theta(getResPose->theta);
            }
            else 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_reeman_pose");
            }
            //Publish
            this->posePublisher->publish(message_pose);
        };
        auto callback_speed = [this]()->void
        {
            auto message_speed = reeman_api_interface::msg::ReemanSpeed();
            std::shared_ptr<rclcpp::Node> node_getReemanSpeed = rclcpp::Node::make_shared("get_reeman_speed_client");
            rclcpp::Client<reeman_api_interface::srv::GetSpeed>::SharedPtr client_getReemanSpeed = node_getReemanSpeed->create_client<reeman_api_interface::srv::GetSpeed>("get_reeman_speed");
            auto req_speed = std::make_shared<reeman_api_interface::srv::GetSpeed::Request>();
            //Get Speed
            while(!client_getReemanSpeed->wait_for_service(100ms)) 
            {
                if (!rclcpp::ok()) 
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }
            auto res_speed = client_getReemanSpeed->async_send_request(req_speed);
            if(rclcpp::spin_until_future_complete(node_getReemanSpeed, res_speed) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto getResSpeed = res_speed.get();
                message_speed.set__vx(getResSpeed->vx);
                message_speed.set__vth(getResSpeed->vth);
            }
            else 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_reeman_speed");
            }
            //Publish
            this->speedPublisher->publish(message_speed);
        };
        timer_pose = this->create_wall_timer(300ms, callback_pose);
        timer_speed = this->create_wall_timer(300ms, callback_speed);
    }

    private:
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr posePublisher;
    rclcpp::Publisher<reeman_api_interface::msg::ReemanSpeed>::SharedPtr speedPublisher;
    rclcpp::TimerBase::SharedPtr timer_pose;
    rclcpp::TimerBase::SharedPtr timer_speed;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<publisher>());
    rclcpp::shutdown();
    return 0;
}