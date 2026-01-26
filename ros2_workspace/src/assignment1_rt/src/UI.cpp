#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "assignment1_rt/msg/velocity_info.hpp"   
#include <iostream>

class UI : public rclcpp::Node {
public:
    UI() : Node("ui_node") {
        // create a publisher for telling which turtle is moving
        moving_turtle_pub_ = this->create_publisher<std_msgs::msg::String>("/moving_turtle", 10); 

        //create a subscriber for stopping turtle moving
        stop_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/stop_movement",
            10,
            std::bind(&UI::stop_callback, this, std::placeholders::_1));

        // NEW: publisher of custom message with velocity info
        velocity_info_pub_ = this->create_publisher<assignment1_rt::msg::VelocityInfo>(
            "/velocity_info",
            10
        );

        while (rclcpp::ok()) {
            std::string turtle; // string that contains the name of the selected turtle

            // ask to the user which turtle select
            std::cout << "Which turtle do you want to control (turtle1/turtle2)? ";
            std::cin >> turtle;

            // validate input
            if (turtle != "turtle1" && turtle != "turtle2") {
                std::cout << "Invalid turtle name! Try again.\n";
                continue;
            }

            // build the cmd_vel topic for the selected turtle (containing the turtle name)
            std::string topic = "/" + turtle + "/cmd_vel";

            // create publisher for this turtle
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(topic, 10);

            // initialize the control variables
            float linear_velocity = 0.0;
            float angular_velocity = 0.0;

            // ask to the user the values of control variables
            std::cout << "Insert linear velocity: ";
            std::cin >> linear_velocity;
            std::cout << "Insert angular velocity: ";
            std::cin >> angular_velocity;

            // initialize the values of the message
            geometry_msgs::msg::Twist message;
            message.linear.x = linear_velocity;
            message.angular.z = angular_velocity;

            // initialize and publish which turtle is moving (for the distance node)
            std_msgs::msg::String msg;
            msg.data = turtle;
            moving_turtle_pub_->publish(msg);

            // sent message for 1 second
            rclcpp::Rate rate(10); // 10 Hz
            auto start = now();

            force_stop_ = false;  // reset before movement

            while (!force_stop_ && (now() - start).seconds() < 1.0 && rclcpp::ok()) {
                // publish the velocity of the moving turtle
                publisher_->publish(message);

                // NEW: publish custom message at EACH ITERATION
                assignment1_rt::msg::VelocityInfo vinfo;
                vinfo.label = "velocity";          // <-- requested string
                vinfo.linear_x = cmd.linear.x;     // <-- requested linear x
                vinfo.angular_z = cmd.angular.z;   // <-- requested angular z
                velocity_info_pub_->publish(vinfo);

                rclcpp::spin_some(this->get_node_base_interface());
                rate.sleep();
            }

            // stop moving after 1 second
            message.linear.x = 0;
            message.angular.z = 0;
            publisher_->publish(message);

            // NEW: also publish the stop velocity on the custom topic (consistent)
            assignment1_rt::msg::VelocityInfo stop_vinfo;
            stop_vinfo.label = "velocity";
            stop_vinfo.linear_x = 0.0f;
            stop_vinfo.angular_z = 0.0f;
            velocity_info_pub_->publish(stop_vinfo);

            std::cout << "Command finished. Insert new command...\n";
        }
    }

private:
    //boolean variable for stop publishing movement
        bool force_stop_ = false;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr moving_turtle_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stop_sub_;

     // NEW: publisher of custom message
    rclcpp::Publisher<assignment1_rt::msg::VelocityInfo>::SharedPtr velocity_info_pub_;

    // callback for setting the stop boolean to true (set by distance_node)
    void stop_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "stop") {
            force_stop_ = true;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UI>();
    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
