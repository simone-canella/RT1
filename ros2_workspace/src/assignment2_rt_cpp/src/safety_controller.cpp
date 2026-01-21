#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>

class SafetyController : public rclcpp::Node {
public:
    SafetyController() : Node("safety_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Safety Controller started");
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafetyController>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
