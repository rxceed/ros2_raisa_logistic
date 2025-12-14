#include "rclcpp/rclcpp.hpp"
#include "custom_interface/srv/pozyx_to_reeman.hpp"
#include "reeman_pose/pozyx_to_reeman_model.h"

#include <memory>

void pozyxToReeman(const std::shared_ptr<custom_interface::srv::PozyxToReeman::Request> req, std::shared_ptr<custom_interface::srv::PozyxToReeman::Response> res)
{
    double pozyxPose[2] = {req->pozyx_x, req->pozyx_y};
    double reemanPose[2];
    predict_all(pozyxPose, reemanPose);
    res->set__reeman_x(reemanPose[0]);
    res->set__reeman_y(reemanPose[1]);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pozyx_to_reeman_server");
    rclcpp::Service<custom_interface::srv::PozyxToReeman>::SharedPtr service = node->create_service<custom_interface::srv::PozyxToReeman>("pozyx_to_reeman", &pozyxToReeman);
    rclcpp::spin(node);
    rclcpp::shutdown();
}