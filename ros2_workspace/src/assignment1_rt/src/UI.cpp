#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>

class UI : public rclcpp::Node {
public:
    UI() : Node("ui_node") {

        while (rclcpp::ok()) {
            std::string turtle; // string that contains the name of the selected turtle

            // ask to the user which turtle select
            std::cout << "Which turtle do you want to control (turtle1/turtle2)? ";
            std::cin >> turtle;

            // validate input
            if (turtle != "turtle1" && turtle != "turtle2") {
                std::cout << "Invalid turtle name! Try again.\n";
                continue;
            }

            // create a topic containing the turtle name 
            std::string topic = "/" + turtle + "/cmd_vel";

            // create publisher for this turtle
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(topic, 10);

            // initialize the control variables
            float linear_velocity = 0.0;
            float angular_velocity = 0.0;

            // ask to the user the values of control variables
            std::cout << "Insert linear velocity: ";
            std::cin >> linear_velocity;
            std::cout << "Insert angular velocity: ";
            std::cin >> angular_velocity;

            // initialize the values of the message
            geometry_msgs::msg::Twist message;
            message.linear.x = linear_velocity;
            message.angular.z = angular_velocity;

            // sent message for 1 second
            rclcpp::Rate rate(10); // 10 Hz
            auto start = now();

            while ((now() - start).seconds() < 1.0 && rclcpp::ok()) {
                publisher_->publish(message);
                rate.sleep();
            }

            // stop moving after 1 second
            message.linear.x = 0;
            message.angular.z = 0;
            publisher_->publish(message);

            std::cout << "Command finished. Insert new command...\n";
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UI>();
    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
