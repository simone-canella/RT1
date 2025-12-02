#include "rclcpp/rclcpp.hpp" 
#include "turtlesim/msg/pose.hpp" // what we want to read (subscribe)
#include "geometry_msgs/msg/twist.hpp" // what we want to write (publish)
#include <iostream>
#include <functional>

using std::placeholders::_1; // use "_1" as container of our message

class Controller : public rclcpp::Node {
public:
    //CONSTRUCTOR
    Controller() : Node("controller"){
        // 1. SUBSCRIBER: read turtle pose
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", // topic name
            10, // queue size
            std::bind(&Controller::topic_callback, this, _1));
        
        // 2. PUBLISHER: send velocity commands
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);
        
        // 3. TIMER: control loop (runs every 100 ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&Controller::timer_callback, this)); 
    }

private:
    // === MEMBERS ===
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    float x_;
    float y_;
    float theta_;

    // === CALLBACKS ===

    // Pose subscriber callback
    void topic_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        x_ = msg->x;
        y_ = msg->y;
        theta_ = msg->theta;

        RCLCPP_INFO(this->get_logger(),
                    "Pose -> x: %.2f  y: %.2f  theta: %.2f",
                    x_, y_, theta_);
    }


    // Control loop (timer)
    void timer_callback()
    {
        geometry_msgs::msg::Twist message;

        // CLOSED-LOOP LOGIC (slide 9)
        if (x_ < 9.0) {
            message.linear.x = 2.0;
            message.angular.z = 0.0;
        } else {
            message.linear.x = 0.0;
            message.angular.z = 0.0;
        }

        publisher_->publish(message);
    }

    //rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
