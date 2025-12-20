#include "reeman_api_service/httpClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "reeman_api_interface/srv/get_nav_status.hpp"
#include "reeman_api_service/secret.hpp"

#include <memory>
#include <nlohmann/json.hpp>

void getReemanPose(const std::shared_ptr<reeman_api_interface::srv::GetNavStatus::Request> req, std::shared_ptr<reeman_api_interface::srv::GetNavStatus::Response> res)
{
    HttpClient client;
    std::string status = client.get(GET_NAV_STATUS_URL);
    auto parsedStatus = nlohmann::json::parse(status);
    res->set__res(parsedStatus["res"]);
    res->set__code(parsedStatus["reason"]);
    res->set__goal(parsedStatus["goal"]);
    res->set__dist(parsedStatus["dist"]);
    res->set__mileage(parsedStatus["mileage"]);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node_getReemanNavStatus = rclcpp::Node::make_shared("get_reeman_nav_status_server");
    rclcpp::Service<reeman_api_interface::srv::GetNavStatus>::SharedPtr service_getReemanNavStatus = node_getReemanNavStatus->create_service<reeman_api_interface::srv::GetNavStatus>("get_reeman_nav_status", &getReemanPose);
    rclcpp::spin(node_getReemanNavStatus);
    rclcpp::shutdown();
}
