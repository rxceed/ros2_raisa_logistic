#include "reeman_api_service/httpClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "reeman_api_interface/srv/post_nav_point.hpp"
#include "reeman_api_service/secret.hpp"

#include <memory>
#include <nlohmann/json.hpp>

void postReemanNavPoint(const std::shared_ptr<reeman_api_interface::srv::PostNavPoint::Request> req, std::shared_ptr<reeman_api_interface::srv::PostNavPoint::Response> res)
{
    HttpClient client;
    nlohmann::json reqBody_json = {{"point", req->point}};
    std::string reqBody_str = reqBody_json.dump();
    std::string response = client.post(POST_NAV_URL, reqBody_str);
    auto parsedResponse = nlohmann::json::parse(response);
    res->set__status(parsedResponse["status"]);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node_postReemanNavPoint = rclcpp::Node::make_shared("post_reeman_nav_point_server");
    rclcpp::Service<reeman_api_interface::srv::PostNavPoint>::SharedPtr service_postReemanNavPoint = node_postReemanNavPoint->create_service<reeman_api_interface::srv::PostNavPoint>("post_reeman_nav_point", &postReemanNavPoint);
    rclcpp::spin(node_postReemanNavPoint);
    rclcpp::shutdown();
}