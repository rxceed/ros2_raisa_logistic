#include <iostream>
#include <curl/curl.h>
#include <memory>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "reeman_pose/httpClient.hpp"
#include <nlohmann/json.hpp>
#include "reeman_pose/secret.hpp"

using namespace std::chrono_literals;

class posePublisher : public rclcpp::Node
{
    public:
    posePublisher() : Node("pose_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("reeman_pose2d", 10);
        auto callback = [this]()->void {
            std::string pose = client.get(GET_POSE_URL);
            auto parsedPose = nlohmann::json::parse(pose);
            auto message = geometry_msgs::msg::Pose2D();
            message.set__x(parsedPose["x"]);
            message.set__y(parsedPose["y"]);
            message.set__theta(parsedPose["theta"]);
            this->publisher_->publish(message);
        };
        timer_ = this->create_wall_timer(300ms, callback);
    }

    private:
    HttpClient client;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<posePublisher>());
    rclcpp::shutdown();
    return 0;
}