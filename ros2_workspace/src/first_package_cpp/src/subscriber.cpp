#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1; //when a message arrives put it in _1

/* MinimalSubscriber Class Definition */
// Inherits from rclcpp::Node to gain all ROS 2 Node functionalities
class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber() : Node("minimal_subscriber")
    {
        // Creates the subscription: watches the "topic" topic
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 
            10, 
            std::bind(&MinimalSubscriber::topic_callback, this, _1)); //use the message contained in "_1" as input parameter of the callback method
    }

private:
    // Callback function: executed every time a new message is received
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

/* Main Function */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); //initializes the entire ROS2 communication system.
    // Starts the blocking loop that waits for incoming messages
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}