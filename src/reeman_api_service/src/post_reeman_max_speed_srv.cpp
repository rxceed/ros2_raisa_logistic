#include "reeman_api_service/httpClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "reeman_api_interface/srv/post_max_speed.hpp"
#include "reeman_api_service/secret.hpp"

#include <memory>
#include <nlohmann/json.hpp>

void postReemanMaxSpeed(const std::shared_ptr<reeman_api_interface::srv::PostMaxSpeed::Request> req, std::shared_ptr<reeman_api_interface::srv::PostMaxSpeed::Response> res)
{
    HttpClient client;
    nlohmann::json reqBody_json = {{"speed", req->speed}};
    std::string reqBody_str = reqBody_json.dump();
    std::string response = client.post(POST_MAX_SPEED_URL, reqBody_str);
    auto parsedResponse = nlohmann::json::parse(response);
    res->set__status(parsedResponse["status"]);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node_postReemanMaxSpeed = rclcpp::Node::make_shared("post_reeman_max_speed_server");
    rclcpp::Service<reeman_api_interface::srv::PostMaxSpeed>::SharedPtr service_postReemanMaxSpeed = node_postReemanMaxSpeed->create_service<reeman_api_interface::srv::PostMaxSpeed>("post_reeman_max_nav", &postReemanMaxSpeed);
    rclcpp::spin(node_postReemanMaxSpeed);
    rclcpp::shutdown();
}
