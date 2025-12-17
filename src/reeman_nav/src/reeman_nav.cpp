#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose2_d.hpp"

#include "custom_interface/msg/dual_leg.hpp"
#include "custom_interface/srv/pozyx_to_reeman.hpp"
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
        auto callback_sub_uwb = [this](geometry_msgs::msg::Pose2D::UniquePtr msg_uwb)->void
        {
            pozyx_x = msg_uwb->x;
            pozyx_y = msg_uwb->y;
            pozyx_theta = msg_uwb->theta;
            printf("POZYX x: %.2f y: %.2f\n", pozyx_x, pozyx_y);
        };
        auto callback_sub_dualLeg = [this](custom_interface::msg::DualLeg::UniquePtr msg_dualLeg)->void
        {
            right_angle = msg_dualLeg->right_angle;
            left_angle = msg_dualLeg->left_angle;
            if(right_angle > 60.0 || left_angle > 60.0)
            {
                post_nav();
            }   
        };
        subscriber_dualLeg = this->create_subscription<custom_interface::msg::DualLeg>("dual_leg", 10, callback_sub_dualLeg);
        subscriber_uwb = this->create_subscription<geometry_msgs::msg::Pose2D>("uwb_pose2d", 10, callback_sub_uwb);
    }
    void post_nav()
    {
        std::shared_ptr<rclcpp::Node> node_postReemanNav_nav = rclcpp::Node::make_shared("post_reeman_nav_nav_client");
        rclcpp::Client<reeman_api_interface::srv::PostNav>::SharedPtr client_postReemanNav_nav = node_postReemanNav_nav->create_client<reeman_api_interface::srv::PostNav>("post_reeman_nav");
        std::shared_ptr<rclcpp::Node> node_pozyxToReeman_nav = rclcpp::Node::make_shared("pozyx_to_reeman_nav_client");
        rclcpp::Client<custom_interface::srv::PozyxToReeman>::SharedPtr client_pozyxToReeman_nav = node_pozyxToReeman_nav->create_client<custom_interface::srv::PozyxToReeman>("pozyx_to_reeman");
        auto req_nav = std::make_shared<reeman_api_interface::srv::PostNav::Request>();
        auto req_pozyxReeman = std::make_shared<custom_interface::srv::PozyxToReeman::Request>();
        req_pozyxReeman->set__pozyx_x(pozyx_x);
        req_pozyxReeman->set__pozyx_y(pozyx_y);

        while (!client_pozyxToReeman_nav->wait_for_service(100ms))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service POZYX TO REEMAN not available, waiting again...");
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

        printf("REEMAN x: %.2f y: %.2f\n", reeman_x, reeman_y);
        
        while (!client_postReemanNav_nav->wait_for_service(100ms))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service POST REEMAN NAV not available, waiting again...");
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
    };

    private:
    rclcpp::Subscription<custom_interface::msg::DualLeg>::SharedPtr subscriber_dualLeg;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subscriber_uwb;
    rclcpp::TimerBase::SharedPtr timer_post_speed;
    float right_angle, left_angle;
    float pozyx_x, pozyx_y, pozyx_theta; //pozyx
    float reeman_x, reeman_y, reeman_theta;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<navigation>());
    rclcpp::shutdown();
    return 0;
}