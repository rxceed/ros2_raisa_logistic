#include "reeman_api_service/httpClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "reeman_api_interface/srv/get_speed.hpp"
#include "reeman_api_service/secret.hpp"

#include <memory>
#include <nlohmann/json.hpp>

void getReemanSpeed(const std::shared_ptr<reeman_api_interface::srv::GetSpeed::Request> req, std::shared_ptr<reeman_api_interface::srv::GetSpeed::Response> res)
{
    HttpClient client;
    std::string speed = client.get(GET_SPEED_URL);
    auto parsedSpeed = nlohmann::json::parse(speed);
    res->set__vx(parsedSpeed["vx"]);
    res->set__vth(parsedSpeed["vth"]);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node_getReemanSpeed = rclcpp::Node::make_shared("get_reeman_speed_server");
    rclcpp::Service<reeman_api_interface::srv::GetSpeed>::SharedPtr service_getReemanSpeed = node_getReemanSpeed->create_service<reeman_api_interface::srv::GetSpeed>("get_reeman_speed", &getReemanSpeed);
    rclcpp::spin(node_getReemanSpeed);
    rclcpp::shutdown();
}