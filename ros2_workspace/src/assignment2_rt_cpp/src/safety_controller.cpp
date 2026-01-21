#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <iostream>

class SafetyController : public rclcpp::Node
{
public:
    SafetyController() : Node("safety_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Safety Controller started");

        // SUBSCRIBERS
        // subscribe to user command
        user_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/user_cmd_vel", 10,
            std::bind(&SafetyController::userCommandCallback, this, std::placeholders::_1));

        // PUBLISHERS
        // publisher to cmd_vel
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Published initial stop on /cmd_vel");

        // TIMER
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&SafetyController::controlLoop, this));
    }

private:
    // variable to store the last command
    geometry_msgs::msg::Twist last_user_cmd_;

    bool have_user_cmd_ = false; // control if the user input is arrived

    // CALLBACKS
    // user command callback:
    void userCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_user_cmd_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Got user cmd: lin.x=%.2f ang.z=%.2f",
                    msg->linear.x, msg->angular.z);

        last_user_cmd_ = *msg;
        have_user_cmd_ = true;
    }

    // timer callback
    void controlLoop()
    {
        if (have_user_cmd_)
        {
            cmd_vel_pub_->publish(last_user_cmd_);
        }
        else
        {
            geometry_msgs::msg::Twist stop;
            cmd_vel_pub_->publish(stop);
        }
    }

    // SUBSCRIBERS
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr user_cmd_sub_;

    // PUBLISHERS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // TIMER
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafetyController>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
