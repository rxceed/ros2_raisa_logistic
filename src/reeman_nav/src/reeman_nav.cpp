#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "custom_interface/msg/dual_leg.hpp"
#include "custom_interface/srv/pozyx_to_reeman.hpp"
#include "custom_interface/srv/logistic_nav_status.hpp"
#include "custom_interface/msg/activity_forecast.hpp"
#include "reeman_api_interface/srv/post_nav_point.hpp"
#include "reeman_api_interface/srv/get_nav_status.hpp"

#include <memory>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

constexpr float GOAL_REACHED_TOL = 0.4f; // meter

class navigation : public rclcpp::Node
{
public:
    navigation() : Node("reeman_navigation_node")
    {
        auto callback_sub_reeman =
            [this](geometry_msgs::msg::Pose2D::UniquePtr msg)->void
        {
            reeman_x = msg->x;
            reeman_y = msg->y;
            reeman_theta = msg->theta;
        };

        auto callback_check = [this]()->void
        {
            get_navStatus();
        };

        auto callback_sub_forecast = [this](custom_interface::msg::ActivityForecast::UniquePtr msg)->void
        {
            int action_index = 0;

            action_forecast[0] = msg->t_plus_1;
            action_forecast[1] = msg->t_plus_2;
            action_forecast[2] = msg->t_plus_3;
            action_forecast[3] = msg->t_plus_4;

            for(int i = 0; i < 4; i++)
            {
                if(action_forecast[i] != 0 && action_forecast[i] != logisticNavStatus)
                {
                    action_index = action_forecast[i];
                    break;
                }
                else
                {
                    action_index = logisticNavStatus;
                }
            }
            if(action_index == logisticNavStatus)
            {
                return;
            }
            if(navStatus_res == 3 && navStatus_code == 0)
            {
                logisticNavStatus_req(1, action_index);
                post_nav(action_index);
            }
        };
        subscriber_forecast =
            create_subscription<custom_interface::msg::ActivityForecast>(
                "action_forecast", 10, callback_sub_forecast);

        subscriber_reeman =
            create_subscription<geometry_msgs::msg::Pose2D>(
                "reeman_pose2d", 10, callback_sub_reeman);
        
        timer_ = create_wall_timer(300ms, callback_check);
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
            logisticNavStatus = getRes_logisticNavStatus->status;
        }
        else 
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service logistic_nav_status");
        }
    }

    void get_navStatus()
    {
        auto node_getNavStatus = rclcpp::Node::make_shared("get_reeman_nav_status_nav_client");
        auto client_getNavStatus = node_getNavStatus->create_client<reeman_api_interface::srv::GetNavStatus>("get_reeman_nav_status");
        auto req = std::make_shared<reeman_api_interface::srv::GetNavStatus::Request>();
        while (!client_getNavStatus->wait_for_service(100ms))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the get_reeman_nav_status service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "get_reeman_nav_status service not available, waiting again...");
        }

        auto res_navStatus = client_getNavStatus->async_send_request(req);
        if(rclcpp::spin_until_future_complete(node_getNavStatus, res_navStatus) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto getRes = res_navStatus.get();
            navStatus_res = getRes->res;
            navStatus_code = getRes->code;
        } 
        else 
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_reeman_nav_status");
        }
    }

    void post_nav(int nav_point)
    {
        std::string point;
        switch(nav_point)
        {
        case 0:
            point = "origin";
            break;
        case 1:
            point = "workbench_1";
            break;
        case 2:
            point = "prod_1";
            break;
        case 3:
            point = "workbench_2";
            break;
        case 4:
            point = "prod_2";
            break;
        case 5:
            point = "workbench_2";
            break;
        case 6:
            point = "origin";
            break;
        default:
            point = "origin";
            break;
        }
        auto node_post =
            rclcpp::Node::make_shared("post_reeman_nav_point_nav_client");
        auto client_post =
            node_post->create_client<
                reeman_api_interface::srv::PostNavPoint>("post_reeman_nav_point");
        auto req_nav =
            std::make_shared<
                reeman_api_interface::srv::PostNavPoint::Request>();
        while (!client_post->wait_for_service(100ms))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the post_reeman_nav_point service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "post_reeman_nav_point service not available, waiting again...");
        }
        
        req_nav->set__point(point);
        auto res_nav = client_post->async_send_request(req_nav);
        if(rclcpp::spin_until_future_complete(node_post, res_nav) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto getRes_nav = res_nav.get();
            std::string status = getRes_nav->status;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "status: %s\n", status);
        } 
        else 
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service logistic_nav_status");
        }
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subscriber_reeman;
    rclcpp::Subscription<custom_interface::msg::ActivityForecast>::SharedPtr subscriber_forecast;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<rclcpp::Node> node_logisticNavStatus = rclcpp::Node::make_shared("logistic_nav_status_client");
    std::shared_ptr<rclcpp::Node> node_navStatus = rclcpp::Node::make_shared("nav_status_nav_client");

    rclcpp::Client<reeman_api_interface::srv::GetNavStatus>::SharedPtr client_navStatus = node_navStatus->
    create_client<reeman_api_interface::srv::GetNavStatus>("get_reeman_nav_status");
    rclcpp::Client<custom_interface::srv::LogisticNavStatus>::SharedPtr client_logisticNavStatus = node_logisticNavStatus->
    create_client<custom_interface::srv::LogisticNavStatus>("logistic_nav_status");

    float reeman_x = 0;
    float reeman_y = 0;
    float reeman_theta = 0;
    char logisticNavStatus = 0;
    int action_forecast[4];
    
    int navStatus_res = 6;
    int navStatus_code = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<navigation>());
    rclcpp::shutdown();
    return 0;
}
