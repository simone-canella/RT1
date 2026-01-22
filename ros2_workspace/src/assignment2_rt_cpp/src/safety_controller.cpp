#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <limits>
#include <cmath>
#include <iostream>

class SafetyController : public rclcpp::Node
{
public:
    SafetyController() : Node("safety_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Safety Controller started");

        // SUBSCRIBERS
        // subscribe to user command
        user_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/user_cmd_vel", 10,
            std::bind(&SafetyController::userCommandCallback, this, std::placeholders::_1));

        scanner_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SafetyController::scannerCallback, this, std::placeholders::_1));

        // PUBLISHERS
        // publisher to cmd_vel
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Published initial stop on /cmd_vel");

        // TIMER
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&SafetyController::controlLoop, this));
    }

private:
    // variable to store the last command
    geometry_msgs::msg::Twist last_user_cmd_;

    bool have_user_cmd_ = false; // control if the user input is arrived

    float min_distance_ = std::numeric_limits<float>::infinity(); // distance of closest obstacle
    uint8_t obstacle_sector_ = 0;                                 // 0=FRONT, 1=LEFT, 2=BACK, 3=RIGHT

    // given an angle normalize it to [-pi, +pi]
    static double normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    // given an angle select in which sector it belongs
    static uint8_t sectorClassifier(double angle)
    {
        // FRONT: [-pi/4, +pi/4]
        if (angle >= -M_PI / 4.0 && angle <= M_PI / 4.0)
            return 0;
        // LEFT:  [pi/4, 3pi/4]
        if (angle > M_PI / 4.0 && angle <= 3.0 * M_PI / 4.0)
            return 1;
        // RIGHT: [-3pi/4, -pi/4]
        if (angle >= -3.0 * M_PI / 4.0 && angle < -M_PI / 4.0)
            return 3;
        // BACK: everything else
        return 2;
    }

    // convert a sector expresseb by an int into a string
    static const char *sectorToString(uint8_t sector)
    {
        switch (sector)
        {
        case 0:
            return "FRONT";
        case 1:
            return "LEFT";
        case 2:
            return "BACK";
        case 3:
            return "RIGHT";
        default:
            return "UNKNOWN";
        }
    }

    // CALLBACKS
    // user command callback:
    void userCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_user_cmd_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Got user cmd: lin.x=%.2f ang.z=%.2f",
                    msg->linear.x, msg->angular.z);

        have_user_cmd_ = true;
    }

    // scanner callback:
    void scannerCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received scan");

        float closest_distance = std::numeric_limits<float>::infinity();
        int closest_distance_index = -1;

        // identify the closest obstacle distance and its index(ranges[])
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float r = msg->ranges[i];
            if (!std::isfinite(r))
                continue;
            if (r < msg->range_min || r > msg->range_max)
                continue;
            if (r < closest_distance)
            {
                closest_distance = r;
                closest_distance_index = (int)i;
            }
        }

        if (closest_distance_index >= 0)
        {
            double angle = msg->angle_min + closest_distance_index * msg->angle_increment;
            angle = normalizeAngle(angle);
            min_distance_ = closest_distance;
            obstacle_sector_ = sectorClassifier(angle);

            RCLCPP_INFO(
                this->get_logger(),
                "Closest obstacle: %.2f m | angle %.2f rad | sector %s",
                min_distance_,
                angle,
                sectorToString(obstacle_sector_));
        }
    }

    // timer callback
    void controlLoop()
    {
        if (have_user_cmd_)
        {
            cmd_vel_pub_->publish(last_user_cmd_);
        }
        else
        {
            geometry_msgs::msg::Twist stop;
            cmd_vel_pub_->publish(stop);
        }
    }

    // SUBSCRIBERS
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr user_cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanner_sub_;

    // PUBLISHERS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // TIMER
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafetyController>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
