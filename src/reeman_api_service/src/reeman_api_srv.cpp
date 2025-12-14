#include "reeman_api_service/httpClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "reeman_api_interface/srv/get_pose.hpp"
#include "reeman_api_service/secret.hpp"

#include <memory>
#include <nlohmann/json.hpp>

void getReemanPose(const std::shared_ptr<reeman_api_interface::srv::GetPose::Request> req, std::shared_ptr<reeman_api_interface::srv::GetPose::Response> res)
{
    HttpClient client;
    std::string pose = client.get(GET_POSE_URL);
    auto parsedPose = nlohmann::json::parse(pose);
    res->set__x(parsedPose["x"]);
    res->set__y(parsedPose["y"]);
    res->set__theta(parsedPose["theta"]);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node_getReemanPose = rclcpp::Node::make_shared("get_reeman_pose_server");
    rclcpp::Service<reeman_api_interface::srv::GetPose>::SharedPtr service_getReemanPose = node_getReemanPose->create_service<reeman_api_interface::srv::GetPose>("get_reeman_pose", &getReemanPose);
    rclcpp::spin(node_getReemanPose);
    rclcpp::shutdown();
}
