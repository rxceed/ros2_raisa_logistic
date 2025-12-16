#include "reeman_api_service/httpClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "reeman_api_interface/srv/post_nav.hpp"
#include "reeman_api_service/secret.hpp"

#include <memory>
#include <nlohmann/json.hpp>

void postReemanNav(const std::shared_ptr<reeman_api_interface::srv::PostNav::Request> req, std::shared_ptr<reeman_api_interface::srv::PostNav::Response> res)
{
    HttpClient client;
    nlohmann::json reqBody_json = {{"x", req->x}, {"y", req->y}, {"theta", req->theta}};
    std::string reqBody_str = reqBody_json.dump();
    std::string response = client.post(POST_NAV_URL, reqBody_str);
    auto parsedResponse = nlohmann::json::parse(response);
    res->set__status(parsedResponse["status"]);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node_postReemanNav = rclcpp::Node::make_shared("post_reeman_nav_server");
    rclcpp::Service<reeman_api_interface::srv::PostNav>::SharedPtr service_postReemanNav = node_postReemanNav->create_service<reeman_api_interface::srv::PostNav>("post_reeman_nav", &postReemanNav);
    rclcpp::spin(node_postReemanNav);
    rclcpp::shutdown();
}