#include "rclcpp/rclcpp.hpp" 
#include "turtlesim/msg/pose.hpp" // what we want to read (subscribe)
#include "geometry_msgs/msg/twist.hpp" // what we want to write (publish)
#include "turtlesim/srv/teleport_absolute.hpp" // for setting the position
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
        // 3. CLIENT: setting the position
        teleport_client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");

        while (!teleport_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for /turtle1/teleport_absolute service...");
        }

        auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        request->x = 2.5; //not "1.0" because the turning condition
        request->y = 1;
        request->theta = 0.0;

        auto result = teleport_client_->async_send_request(request);

        // 4. TIMER: control loop (runs every 100 ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&Controller::timer_callback, this)); 
        
        turning_ = false;
        moving_right_ = true;
    }

private:
    // === MEMBERS ===
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_; // client for setting position

    //STATE VARIABLES
    float x_ = 0;
    float y_ = 0;
    float theta_ = 0;

    bool turning_; // true = executing a semicircle
    bool moving_right_ ; // false = moving left

    //CONTROL PARAMETERS
    float v_ = 2.0;
    float omega_ = 1.0;

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


        if (y_ > 10.5) { //stop when reach top limit
            publisher_->publish(geometry_msgs::msg::Twist());
            RCLCPP_INFO(this->get_logger(), "Stop: reached top limit");
            return;
        }
        
        if (turning_) { //turning logic
            if (moving_right_) {
                if (theta_ < 3.1) { //anti-clock turning
                    message.linear.x = v_;
                    message.angular.z = omega_;
                } else {
                    turning_ = false; // finished turn
                    moving_right_ = false; // now going left
                }
            } else {
                if (theta_ > 0.1) { //clock turning
                    message.linear.x = v_;
                    message.angular.z = -omega_;
                } else {
                    turning_ = false; // finished turn
                    moving_right_ = true; // now going left
                }
            }
        } else { //straight motion
            message.linear.x = v_;
            message.angular.z = 0.0;

            if (moving_right_ && x_ > 9.0) { // right boundary reached
                turning_ = true; //start turning logic
            }

            if (!moving_right_ && x_ < 2.0) { // left boundary reached
                turning_ = true; //start turning logic
            }
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
