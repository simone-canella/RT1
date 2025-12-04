#include "rclcpp/rclcpp.hpp" 
#include "turtlesim/msg/pose.hpp" // what we want to read (subscribe)
#include "geometry_msgs/msg/twist.hpp" // what we want to write (publish)
#include "turtlesim/srv/teleport_absolute.hpp" // for setting the position
#include <iostream>
#include <functional>
#include <cmath>

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

        auto start_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        start_request->x = 2.5; //not "1.0" because the turning condition
        start_request->y = 1;
        start_request->theta = 0.0;
        teleport_client_->async_send_request(start_request);

        // 4. TIMER: control loop (runs every 100 ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&Controller::timer_callback, this)); 
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

    int state = 0;

    //CONTROL PARAMETERS
    float linear_velocity = 1.0;
    float angular_velocity = 1.0;

    // === CALLBACKS ===
    // Pose subscriber callback
    void topic_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        x_ = msg->x;
        y_ = msg->y;
        theta_ = msg->theta;        
    }

    // Control loop (timer)
    void timer_callback()
    {
        geometry_msgs::msg::Twist message;

        if(y_ >= 11.0) {
            RCLCPP_INFO(this->get_logger(), "STOP: top limit reached");
            message.linear.x = 0;
            message.angular.z = 0;
            publisher_->publish(message);
            
            return;
        }
        
        switch(state) {

        case 0:  // MOVE RIGHT
            // set velocities
            message.linear.x = linear_velocity;
            message.angular.z = 0;

            // check if x > 9 → change state
            if (x_ >= 9 ) {
                RCLCPP_INFO(this->get_logger(),
                    "Pose -> x: %.2f  y: %.2f  theta: %.2f",
                    x_, y_, theta_);
                RCLCPP_INFO(this->get_logger(), "CHANGE STATE");
                message.linear.x = 0;
                message.angular.z = 0;
                state++;
            }
            break;

        case 1:  // TURN LEFT
            // set angular velocity
            message.linear.x = linear_velocity;
            message.angular.z = angular_velocity;

            // check if x < 9 → change state
            if (fabs(theta_ - 3.14) < 0.1 || x_ <= 9.0) {
                RCLCPP_INFO(this->get_logger(),
                    "Pose -> x: %.2f  y: %.2f  theta: %.2f",
                    x_, y_, theta_);
                RCLCPP_INFO(this->get_logger(), "CHANGE STATE");

                auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
                request->x = 9.0f; //not "1.0" because the turning condition
                request->y = y_;
                request->theta = 3.14f;

                teleport_client_->async_send_request(request);


                message.linear.x = 0;
                message.angular.z = 0;
                state++;
            }
            break;

        case 2:  // MOVE LEFT
            // set velocities
            message.linear.x = linear_velocity;
            message.angular.z = 0;
            // check if x < 2 → change state
            break;

        case 3:  // TURN RIGHT
            // set angular velocity
            // check if x > 2 → change state
            break;
        }

        publisher_->publish(message);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
