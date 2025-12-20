#include "rclcpp/rclcpp.hpp"
#include "custom_interface/srv/logistic_nav_status.hpp"
#include <memory>

void logisticNavStatus(const std::shared_ptr<custom_interface::srv::LogisticNavStatus::Request> req, std::shared_ptr<custom_interface::srv::LogisticNavStatus::Response> res)
{
    //status
    //0: idle
    //1: go to workbench_1
    //2: go to prod_1
    //3: go to workbench_2
    //4: go to prod_2
    //5: go to workbench_2
    //6: go to last point
    static char logisticNavStatus = 0;
    //action
    //0: get
    //1: post
    if(req->action == 0)
    {
        res->set__status(logisticNavStatus);
    }
    else if(req->action == 1)
    {
        logisticNavStatus = req->state;
        res->set__status(logisticNavStatus);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("logistic_nav_status_server");
    rclcpp::Service<custom_interface::srv::LogisticNavStatus>::SharedPtr service = node->create_service<custom_interface::srv::LogisticNavStatus>("logistic_nav_status", &logisticNavStatus);
    rclcpp::spin(node);
    rclcpp::shutdown();
}
