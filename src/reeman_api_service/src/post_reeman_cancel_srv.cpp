#include "reeman_api_service/httpClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "reeman_api_interface/srv/post_cancel.hpp"
#include "reeman_api_service/secret.hpp"

#include <memory>
#include <nlohmann/json.hpp>

void postReemanCancel(const std::shared_ptr<reeman_api_interface::srv::PostCancel::Request> req, std::shared_ptr<reeman_api_interface::srv::PostCancel::Response> res)
{
    HttpClient client;
    std::string reqBody_str = "{}";
    std::string response = client.post(POST_CANCEL_NAV_URL, reqBody_str);
    auto parsedResponse = nlohmann::json::parse(response);
    res->set__status(parsedResponse["status"]);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node_postReemanCancel = rclcpp::Node::make_shared("post_reeman_cancel_server");
    rclcpp::Service<reeman_api_interface::srv::PostCancel>::SharedPtr service_postReemanCancel = node_postReemanCancel->create_service<reeman_api_interface::srv::PostCancel>("post_reeman_cancel", &postReemanCancel);
    rclcpp::spin(node_postReemanCancel);
    rclcpp::shutdown();
}