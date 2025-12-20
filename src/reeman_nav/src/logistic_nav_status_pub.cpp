#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>
#include <chrono>
#include "custom_interface/srv/logistic_nav_status.hpp"

using namespace std::chrono_literals;

class logicticNavStatusPub : public rclcpp::Node
{
    public:
    logicticNavStatusPub() : Node("logistiv_nav_status_publisher")
    {
        logisticNavStatusPublisher = this->create_publisher<std_msgs::msg::Char>("logistic_nav_status", 10);
        auto callback_logistic_nav_status = [this]()->void{
            auto logNavStat_msg = std_msgs::msg::Char();
            logisticNavStatus_req(0,0);
            logNavStat_msg.set__data(logNavStat);
            this->logisticNavStatusPublisher->publish(logNavStat_msg);
        };
        timer_ = this->create_wall_timer(1s, callback_logistic_nav_status);
    }
    void logisticNavStatus_req(char action, char state)
    {
        auto req_logisticNavStatus = std::make_shared<custom_interface::srv::LogisticNavStatus::Request>();

        req_logisticNavStatus->set__action(action);
        req_logisticNavStatus->set__state(state);
        while (!client_logisticNavStatus->wait_for_service(100ms))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the logistic_nav_status service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service logistic_nav_status not available, waiting again...");
        }
        auto res_logisticNavStatus = client_logisticNavStatus->async_send_request(req_logisticNavStatus);
        if(rclcpp::spin_until_future_complete(node_logisticNavStatus, res_logisticNavStatus) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto getRes_logisticNavStatus = res_logisticNavStatus.get();
            logNavStat = getRes_logisticNavStatus->status;
        }
        else 
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service logistic_nav_status");
        }
    }

    private:
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr logisticNavStatusPublisher;
    std::shared_ptr<rclcpp::Node> node_logisticNavStatus = rclcpp::Node::make_shared("logistic_nav_status_pub_client");
    rclcpp::Client<custom_interface::srv::LogisticNavStatus>::SharedPtr client_logisticNavStatus = node_logisticNavStatus->
    create_client<custom_interface::srv::LogisticNavStatus>("logistic_nav_status");
    rclcpp::TimerBase::SharedPtr timer_;
    char logNavStat = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<logicticNavStatusPub>());
    rclcpp::shutdown();
    return 0;
}