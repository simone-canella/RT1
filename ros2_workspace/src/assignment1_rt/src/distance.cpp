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

        // === PUBLISHERS ===
        // publisher for the distance between turtles
        distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("/turtles_distance", 10);

        // publishers to stop turtles
        turtle1_stop_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        turtle2_stop_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);
        
        // === TIMER ===
        // timer for periodic distance computation and safety checks (10 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DistanceNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Distance node started.");
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
    
    // timer callback: where live distance computation + safety logic
    void timer_callback()
    {
        // compute distance between the two turtle
        float distance = sqrt(pow(t1_x_ - t2_x_, 2) + pow(t1_y_ - t2_y_, 2));

        // initialize and publish the distance
        std_msgs::msg::Float32 message;
        message.data = distance;
        distance_pub_ -> publish(message);

        // check thresholds (distance + boundaries) 
        if (moving_turtle_ == "turtle1" && ((t1_x_ < min_boundary_ || 
                                            t1_x_ > max_boundary_ || 
                                            t1_y_ < min_boundary_ || 
                                            t1_y_ > max_boundary_)|| 
                                            distance < distance_threshold_)) {
            
            // stop the moving turtle
            geometry_msgs::msg::Twist stop_message;
            stop_message.linear.x = 0;
            stop_message.angular.z = 0; 

            turtle1_stop_pub_->publish(stop_message);

            RCLCPP_WARN(this->get_logger(), 
                "TURTLE STOP: turtle1 is stopped beacuse to cloose to other turtle or to the boundaries");

        } else if (moving_turtle_ == "turtle2" && ((t2_x_ < min_boundary_ || 
                                                    t2_x_ > max_boundary_ || 
                                                    t2_y_ < min_boundary_ || 
                                                    t2_y_ > max_boundary_)||
                                                    distance < distance_threshold_)) {

            // stop the moving turtle
            geometry_msgs::msg::Twist stop_message;
            stop_message.linear.x = 0;
            stop_message.angular.z = 0; 

            turtle2_stop_pub_->publish(stop_message);

            RCLCPP_WARN(this->get_logger(), 
                "TURTLE STOP: turtle2 is stopped beacuse to cloose to other turtle or to the boundaries");
        }
    }

    // subscribers
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr t1_pose_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr t2_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr moving_turtle_sub_;

    // publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle1_stop_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle2_stop_pub_;

    // timer
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistanceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
