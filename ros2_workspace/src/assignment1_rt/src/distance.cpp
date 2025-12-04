#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <cmath>

class DistanceNode : public rclcpp::Node {
public:
    DistanceNode() : Node("distance_node")
    {
        // === SUBSCRIBERS ===
        // subscriber to turtle1 pose
        t1_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",
            10,
            std::bind(&DistanceNode::t1_pose_callback, this, std::placeholders::_1)
        );

        // subscriber to turtle2 pose
        t2_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle2/pose",
            10,
            std::bind(&DistanceNode::t2_pose_callback, this, std::placeholders::_1)
        );

        // subscriber to topic which tells us which turtle is moving
        moving_turtle_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/moving_turtle",
            10,
            std::bind(&DistanceNode::moving_turtle_callback, this, std::placeholders::_1)
        );

    }

private:
    // pose variables
    float t1_x_ = 0.0f, t1_y_ = 0.0f;
    float t2_x_ = 0.0f, t2_y_ = 0.0f;

    // name of the turtle currently moving ("turtle1" or "turtle2")
    std::string moving_turtle_ = "";

    // thresholds
    float distance_threshold_ = 2.0f; 
    float min_boundary_ = 1.0f;
    float max_boundary_ = 10.0f;

    // === CALLBACKS ===

    // callback for turtle1 pose
    void t1_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        t1_x_ = msg->x;
        t1_y_ = msg->y;
    }

    // callback for turtle2 pose
    void t2_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        t2_x_ = msg->x;
        t2_y_ = msg->y;
    }

    // callback for moving turtle topic
    void moving_turtle_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        moving_turtle_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Currently moving turtle: %s", moving_turtle_.c_str());
    }
    

    // subscribers
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr t1_pose_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr t2_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr moving_turtle_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistanceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
