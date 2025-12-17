#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>

class logictivNavStatusPub : public rclcpp::Node
{
    public:
    logictivNavStatusPub() : Node("logistiv_nav_status_publisher")
    {
        logisticNavStatusPublisher = this->create_publisher<std_msgs::msg::Char>("logistic_nav_status", 10);
        auto callback_logistiv_nav_status = [this]->void{
            
        };
    }

    private:
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr logisticNavStatusPublisher;
    rclcpp::TimerBase::SharedPtr timer_;
};