#include "rclcpp/rclcpp.hpp" 
#include "std_msgs/msg/string.hpp" 
#include <iostream>

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher() : Node("minimal_publisher"){
        // 1. Initialize the PUBLISHER object
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

        // 2. Create the TIMER (replaces the while loop)
        // Calls the function MinimalPublisher::timer_callback every 500ms (0.5 seconds)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MinimalPublisher::timer_callback, this)); 

        count_ = 0;
    }
private:
    // We define the timer_callback() here
    void timer_callback() {
        message.data = "Hello, ROS2! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()); 
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_; 
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; 
    int count_; 
    std_msgs::msg::String message;
};

int main(int argc, char * argv[]) { 
    rclcpp::init(argc, argv); //initializes the entire ROS2 communication system.
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();  
    return 0;
}