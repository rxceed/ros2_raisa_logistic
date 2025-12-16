#include "reeman_api_service/httpClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "reeman_api_interface/srv/post_speed.hpp"
#include "reeman_api_service/secret.hpp"

#include <memory>
#include <nlohmann/json.hpp>

void postReemanSpeed(const std::shared_ptr<reeman_api_interface::srv::PostSpeed::Request> req, std::shared_ptr<reeman_api_interface::srv::PostSpeed::Response> res)
{
    HttpClient client;
    nlohmann::json reqBody_json = {{"vx", req->vx}, {"vth", req->vth}};
    std::string reqBody_str = reqBody_json.dump();
    std::string response = client.post(POST_SPEED_URL, reqBody_str);
    auto parsedResponse = nlohmann::json::parse(response);
    res->set__status(parsedResponse["status"]);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node_postReemanSpeed = rclcpp::Node::make_shared("post_reeman_speed_server");
    std::shared_ptr<rclcpp::Node> node_postReemanMaxSpeed = rclcpp::Node::make_shared("post_reeman_max_speed_server");
    rclcpp::Service<reeman_api_interface::srv::PostSpeed>::SharedPtr service_postReemanSpeed = node_postReemanSpeed->create_service<reeman_api_interface::srv::PostSpeed>("post_reeman_speed", &postReemanSpeed);
    rclcpp::spin(node_postReemanSpeed);
    rclcpp::shutdown();
}