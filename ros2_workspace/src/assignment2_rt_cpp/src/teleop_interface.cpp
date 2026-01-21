#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>

class TeleopInterface : public rclcpp::Node
{
public:
    TeleopInterface() : Node("teleop_interface")
    {
        // Create publisher
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/user_cmd_vel", 10);
    }

    void run()
    {
        while (rclcpp::ok())
        {
            // initialize control variables
            double linear_velocity = 0.0, angular_velocity = 0.0;

            // ask to the user the values of controls variables
            std::cout << "Insert linear velocity: ";
            std::cin >> linear_velocity;
            std::cout << "Insert angular velocity: ";
            std::cin >> angular_velocity;

            // initialize the values of the message
            geometry_msgs::msg::Twist msg;
            msg.linear.x = linear_velocity;
            msg.angular.z = angular_velocity;

            // send message for 1 second
            rclcpp::Rate rate(10);
            auto start = now();

            while ((now() - start).seconds() < 1.0)
            {
                pub_->publish(msg);
                rclcpp::spin_some(this->get_node_base_interface());
                rate.sleep();
            }

            // stop moving after 1 second
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;

            pub_->publish(msg);

            std::cout << "Command finished. Insert new command...\n";
        }
    }

private:
    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopInterface>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
