#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose2_d.hpp"

#include "custom_interface/msg/dual_leg.hpp"
#include "custom_interface/srv/pozyx_to_reeman.hpp"
#include "custom_interface/srv/logistic_nav_status.hpp"
#include "custom_interface/msg/activity_forecast.hpp"
#include "reeman_api_interface/srv/post_speed.hpp"
#include "reeman_api_interface/srv/post_nav.hpp"

#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class navigation : public rclcpp::Node
{
    public:
    navigation() : Node("reeman_navigation_node")
    {
        auto callback_sub_reeman = [this](geometry_msgs::msg::Pose2D::UniquePtr msg_reeman)->void
        {
            odom_x = msg_reeman->x;
            odom_y = msg_reeman->y;
            odom_theta = msg_reeman->theta;
            if(std::abs(target_x-odom_x) <= 0.15 && std::abs(target_y-odom_y) <= 0.15)
            {
                auto req_logisticNavStatus = std::make_shared<custom_interface::srv::LogisticNavStatus::Request>();
                req_logisticNavStatus->set__action(1);
                req_logisticNavStatus->set__state(2);
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
                }
                else 
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service logistic_nav_status");
                }
            }
        };
        auto callback_sub_uwb = [this](geometry_msgs::msg::Pose2D::UniquePtr msg_uwb)->void
        {
            pozyx_x = msg_uwb->x;
            pozyx_y = msg_uwb->y;
            pozyx_theta = msg_uwb->theta;
        };
        auto callback_sub_dualLeg = [this](custom_interface::msg::DualLeg::UniquePtr msg_dualLeg)->void
        {
            right_angle = msg_dualLeg->right_angle;
            left_angle = msg_dualLeg->left_angle;
            /*
            if(right_angle > 60.0 && left_angle > 60.0)
            {
                post_nav();
            }*/
        };
        auto callback_sub_forecast = [this](custom_interface::msg::ActivityForecast::UniquePtr msg_forecast)->void
        {
            char logisticNavStatus;
            for(int i = 0; i < 4; i++)
            {
                forecast_actions[i] = msg_forecast->actions[i];
                forecast_confidences[i] = msg_forecast->confidences[i];
            }
            time_to_arrival = msg_forecast->time_to_arrival;
            auto req_logisticNavStatus = std::make_shared<custom_interface::srv::LogisticNavStatus::Request>();

            req_logisticNavStatus->set__action(0);
            req_logisticNavStatus->set__state(1);
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
                logisticNavStatus = getRes_logisticNavStatus->status;
            }
            else 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service logistic_nav_status");
            }

            if(time_to_arrival < 6.0 && logisticNavStatus == 0)
            {
                post_nav();
            }
        };
        subscriber_dualLeg = this->create_subscription<custom_interface::msg::DualLeg>("dual_leg", 10, callback_sub_dualLeg);
        subscriber_uwb = this->create_subscription<geometry_msgs::msg::Pose2D>("uwb_pose2d", 10, callback_sub_uwb);
        subscriber_forecast = this->create_subscription<custom_interface::msg::ActivityForecast>("activity_forecast", 10, callback_sub_forecast);
        subscriber_reeman = this->create_subscription<geometry_msgs::msg::Pose2D>("reeman_pose2d", 10, callback_sub_reeman);
    }
    void post_nav()
    {
        std::shared_ptr<rclcpp::Node> node_postReemanNav_nav = rclcpp::Node::make_shared("post_reeman_nav_nav_client");
        rclcpp::Client<reeman_api_interface::srv::PostNav>::SharedPtr client_postReemanNav_nav = node_postReemanNav_nav->create_client<reeman_api_interface::srv::PostNav>("post_reeman_nav");
        
        std::shared_ptr<rclcpp::Node> node_pozyxToReeman_nav = rclcpp::Node::make_shared("pozyx_to_reeman_nav_client");
        rclcpp::Client<custom_interface::srv::PozyxToReeman>::SharedPtr client_pozyxToReeman_nav = node_pozyxToReeman_nav->create_client<custom_interface::srv::PozyxToReeman>("pozyx_to_reeman");
        auto req_nav = std::make_shared<reeman_api_interface::srv::PostNav::Request>();
        auto req_pozyxReeman = std::make_shared<custom_interface::srv::PozyxToReeman::Request>();
        auto req_logisticNavStatus = std::make_shared<custom_interface::srv::LogisticNavStatus::Request>();

        req_logisticNavStatus->set__action(1);
        req_logisticNavStatus->set__state(2);

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
        }
        else 
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service logistic_nav_status");
        }

        req_pozyxReeman->set__pozyx_x(pozyx_x);
        req_pozyxReeman->set__pozyx_y(pozyx_y);

        while (!client_pozyxToReeman_nav->wait_for_service(100ms))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the pozyx_to_reeman service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service pozyx_to_reeman not available, waiting again...");
        }
        auto res_pozyxReeman = client_pozyxToReeman_nav->async_send_request(req_pozyxReeman);
        if(rclcpp::spin_until_future_complete(node_pozyxToReeman_nav, res_pozyxReeman) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto getRes_pozyxReeman = res_pozyxReeman.get();
            reeman_x = getRes_pozyxReeman->reeman_x;
            reeman_y = getRes_pozyxReeman->reeman_y;
        }
        else 
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service pozyx_to_reeman");
        }

        req_nav->set__x(reeman_x);
        req_nav->set__y(reeman_y);
        req_nav->set__theta(0);
        
        while (!client_postReemanNav_nav->wait_for_service(100ms))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the post_reeman_nav service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service post_reeman_nav not available, waiting again...");
        }
        auto res_postReemanNav = client_postReemanNav_nav->async_send_request(req_nav);
        if(rclcpp::spin_until_future_complete(node_postReemanNav_nav, res_postReemanNav) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto getRes_postNav = res_postReemanNav.get();
            std::string status = getRes_postNav->status;
        }
        else 
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service post_reeman_nav");
        }

        target_x = reeman_x;
        target_y = reeman_y;
        target_theta = reeman_theta;
    };

    private:
    rclcpp::Subscription<custom_interface::msg::DualLeg>::SharedPtr subscriber_dualLeg;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subscriber_uwb;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subscriber_reeman;
    rclcpp::Subscription<custom_interface::msg::ActivityForecast>::SharedPtr subscriber_forecast;
    std::shared_ptr<rclcpp::Node> node_logisticNavStatus = rclcpp::Node::make_shared("logistic_nav_status_nav_client");
    rclcpp::Client<custom_interface::srv::LogisticNavStatus>::SharedPtr client_logisticNavStatus = node_logisticNavStatus->create_client<custom_interface::srv::LogisticNavStatus>("logistic_nav_status");
    rclcpp::TimerBase::SharedPtr timer_post_speed;
    float right_angle, left_angle;
    float pozyx_x, pozyx_y, pozyx_theta; //pozyx
    float reeman_x, reeman_y, reeman_theta;
    float target_x, target_y, target_theta;
    float odom_x, odom_y, odom_theta;
    std::string forecast_actions[4];
    float forecast_confidences[4];
    float time_to_arrival;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<navigation>());
    rclcpp::shutdown();
    return 0;
}