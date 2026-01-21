#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("teleop_interface");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
