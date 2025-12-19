#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "custom_interface/msg/dual_leg.hpp"
#include "custom_interface/srv/pozyx_to_reeman.hpp"
#include "custom_interface/srv/logistic_nav_status.hpp"
#include "custom_interface/msg/activity_forecast.hpp"
#include "reeman_api_interface/srv/post_nav_point.hpp"

#include <memory>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

constexpr float GOAL_REACHED_TOL = 0.4f; // meter

typedef struct{
    float x;
    float y;
    float theta;
} pose;

class navigation : public rclcpp::Node
{
public:
    navigation() : Node("reeman_navigation_node")
    {
        auto callback_sub_reeman =
            [this](geometry_msgs::msg::Pose2D::UniquePtr msg)->void
        {
            odom_x = msg->x;
            odom_y = msg->y;
            odom_theta = msg->theta;

            check_goal_reached();
        };

        auto callback_sub_uwb =
            [this](geometry_msgs::msg::Pose2D::UniquePtr msg)->void
        {
            pozyx_x = msg->x;
            pozyx_y = msg->y;
            pozyx_theta = msg->theta;
        };

        auto callback_sub_dualLeg =
            [this](custom_interface::msg::DualLeg::UniquePtr msg)->void
        {
            right_angle = msg->right_angle;
            left_angle = msg->left_angle;
        };

        auto callback_sub_forecast =
            [this](custom_interface::msg::ActivityForecast::UniquePtr msg)->void
        {
            for(int i = 0; i < 4; i++)
            {
                forecast_actions[i] = msg->actions[i];
                forecast_confidences[i] = msg->confidences[i];
            }
            time_to_arrival = msg->time_to_arrival;

            // Jika sedang navigasi → abaikan inference baru
            if (nav_active)
            {
                RCLCPP_INFO(get_logger(),
                    "Navigation active (state %d). Ignoring new inference.",
                    active_state);
                return;
            }

            int requested_state = forecast_actions[0]; // t+1

            if (requested_state == 0)
            {
                RCLCPP_INFO(get_logger(),
                    "State 0 received. Robot hold.");
                return;
            }

            char logisticNavStatus = 0;
            auto req_status =
                std::make_shared<custom_interface::srv::LogisticNavStatus::Request>();
            req_status->set__action(0);
            req_status->set__state(1);

            auto res_status =
                client_logisticNavStatus->async_send_request(req_status);

            if (rclcpp::spin_until_future_complete(
                    node_logisticNavStatus, res_status) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(get_logger(),
                    "Failed to call logistic_nav_status");
                return;
            }

            logisticNavStatus = res_status.get()->status;

            pose target = goal_map[requested_state];

            float dist = std::sqrt(
                (odom_x - target.x)*(odom_x - target.x) +
                (odom_y - target.y)*(odom_y - target.y)
            );

            if (time_to_arrival < 12.0 * dist && logisticNavStatus == 0)
            {
                post_nav(target);
                nav_active = true;
                active_state = requested_state;
                active_goal = target;

                RCLCPP_INFO(get_logger(),
                    "Navigation LOCKED to state %d", active_state);
            }
        };

        subscriber_dualLeg =
            create_subscription<custom_interface::msg::DualLeg>(
                "dual_leg", 10, callback_sub_dualLeg);

        subscriber_uwb =
            create_subscription<geometry_msgs::msg::Pose2D>(
                "uwb_pose2d", 10, callback_sub_uwb);

        subscriber_forecast =
            create_subscription<custom_interface::msg::ActivityForecast>(
                "activity_forecast", 10, callback_sub_forecast);

        subscriber_reeman =
            create_subscription<geometry_msgs::msg::Pose2D>(
                "reeman_pose2d", 10, callback_sub_reeman);
    }

    void check_goal_reached()
    {
        if (!nav_active)
            return;

        float dist = std::sqrt(
            (odom_x - active_goal.x)*(odom_x - active_goal.x) +
            (odom_y - active_goal.y)*(odom_y - active_goal.y)
        );

        if (dist < GOAL_REACHED_TOL)
        {
            nav_active = false;
            active_state = -1;

            RCLCPP_INFO(get_logger(),
                "Goal reached. Navigation UNLOCKED.");
        }
    }

    void post_nav(const pose& target)
    {
        auto node_post =
            rclcpp::Node::make_shared("post_reeman_nav_client");
        auto client_post =
            node_post->create_client<
                reeman_api_interface::srv::PostNav>("post_reeman_nav");

        auto req_nav =
            std::make_shared<
                reeman_api_interface::srv::PostNav::Request>();

        req_nav->set__x(target.x);
        req_nav->set__y(target.y);
        req_nav->set__theta(target.theta);

        client_post->async_send_request(req_nav);
    }

private:
    rclcpp::Subscription<custom_interface::msg::DualLeg>::SharedPtr subscriber_dualLeg;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subscriber_uwb;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subscriber_reeman;
    rclcpp::Subscription<custom_interface::msg::ActivityForecast>::SharedPtr subscriber_forecast;

    std::shared_ptr<rclcpp::Node> node_logisticNavStatus =
        rclcpp::Node::make_shared("logistic_nav_status_client");

    rclcpp::Client<custom_interface::srv::LogisticNavStatus>::SharedPtr
        client_logisticNavStatus =
            node_logisticNavStatus->create_client<
                custom_interface::srv::LogisticNavStatus>(
                    "logistic_nav_status");

    float right_angle = 0.0, left_angle = 0.0;
    float pozyx_x = 0.0, pozyx_y = 0.0, pozyx_theta = 0.0;
    float odom_x = 0.0, odom_y = 0.0, odom_theta = 0.0;

    int forecast_actions[4];
    float forecast_confidences[4];
    float time_to_arrival = 0.0;

    bool nav_active = false;
    int active_state = -1;
    pose active_goal;

    pose goal_map[7] = {
        {0.0,  0.0,  0.0},
        {3.51, 3.01, 1.57},
        {3.28,-5.07, 1.57},
        {7.64, 2.76, 1.76},
        {6.13,-4.79, 1.57},
        {7.64, 2.76, 1.76},
        {0.0,  0.0,  0.0}
    };
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<navigation>());
    rclcpp::shutdown();
    return 0;
}
