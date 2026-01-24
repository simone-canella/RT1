#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <limits>
#include <cmath>
#include <tf2/utils.h>
#include <iostream>

class SafetyController : public rclcpp::Node
{
public:
    SafetyController() : Node("safety_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Safety Controller started");

        // default parameters
        this->declare_parameter<double>("threshold", 0.8);
        threshold_ = this->get_parameter("threshold").as_double();

        this->declare_parameter<bool>("debug", false);
        this->declare_parameter<double>("debug_period", 1.0);
        debug_mode_ = this->get_parameter("debug").as_bool();
        debug_period_ = this->get_parameter("debug_period").as_double();
        last_debug_time_ = this->now();

        // SUBSCRIBERS
        // subscribe to user command
        user_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/user_cmd_vel", 10,
            std::bind(&SafetyController::userCommandCallback, this, std::placeholders::_1));

        scanner_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SafetyController::scannerCallback, this, std::placeholders::_1));

        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&SafetyController::odomCallback, this, std::placeholders::_1));

        // PUBLISHERS
        // publisher to cmd_vel
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "cmd_vel publisher created");

        // TIMER
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&SafetyController::controlLoop, this));
    }

private:
    // DEBUG FEATURES
    bool debug_mode_ = false;
    double debug_period_ = 1.0;
    rclcpp::Time last_debug_time_;

    // VARIABLES + CONTROLS STATE
    // variable to store the last command
    geometry_msgs::msg::Twist last_user_cmd_;

    bool have_user_cmd_ = false; // control if the user input is arrived

    float min_distance_ = std::numeric_limits<float>::infinity(); // distance of closest obstacle
    uint8_t obstacle_sector_ = 0;                                 // 0=FRONT, 1=LEFT, 2=BACK, 3=RIGHT

    double current_x_ = 0.0;
    double current_y_ = 0.0;
    bool have_odom_ = false; // control if the odom is arrived

    double yaw_ = 0.0;
    double threshold_ = 0.8; // default (meters)

    double safe_x_ = 0.0;
    double safe_y_ = 0.0;
    double safe_yaw_ = 0.0;
    bool have_safe_pose_ = false; // control if the safe pose is arrived

    bool recovery_mode_ = false; // control if the recovery mode is activated
    bool is_moving_cmd_ = false; // control if the robot is moving

    // INTERNAL FUNCTIONS
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

    // convert a sector expressed by an int into a string
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

    // control if the obstacle is in forward position
    bool isCommandTowardObstacle() const
    {
        const double v = last_user_cmd_.linear.x;
        const double w = last_user_cmd_.angular.z;

        switch (obstacle_sector_)
        {
        case 0:
            return v > 0.0; // FRONT
        case 2:
            return v < 0.0; // BACK
        case 1:
            return w > 0.0; // LEFT
        case 3:
            return w < 0.0; // RIGHT
        default:
            return false;
        }
    }

    // CALLBACKS
    // user command callback:
    void userCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_user_cmd_ = *msg;
        have_user_cmd_ = true;

        // set that the robot is moving
        is_moving_cmd_ = (std::fabs(msg->linear.x) > 1e-3) || (std::fabs(msg->angular.z) > 1e-3);

        if (debug_mode_)
        {
            RCLCPP_INFO(this->get_logger(), "Got user cmd: lin.x=%.2f ang.z=%.2f",
                        msg->linear.x, msg->angular.z);
        }
    }

    // scanner callback:
    void scannerCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
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
        }
    }

    // odometry callback: odometry orientation is a quaternion, we use tf2 for extracting yaw,pitch,roll
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!have_safe_pose_)
        {
            safe_x_ = current_x_;
            safe_y_ = current_y_;
            safe_yaw_ = yaw_;
            have_safe_pose_ = true;
        }

        yaw_ = tf2::getYaw(msg->pose.pose.orientation);

        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        have_odom_ = true;
    }

    // timer callback: where decisions are taken
    void controlLoop()
    {
        if (!recovery_mode_ && have_safe_pose_ && have_user_cmd_ && std::isfinite(min_distance_) && min_distance_ < threshold_ && isCommandTowardObstacle())
        {
            recovery_mode_ = true;
        }

        if (have_user_cmd_)
        {
            cmd_vel_pub_->publish(last_user_cmd_);
        }
        else
        {
            geometry_msgs::msg::Twist stop;
            cmd_vel_pub_->publish(stop);
        }

        if (have_odom_ && std::isfinite(min_distance_) && min_distance_ >= threshold_)
        {
            safe_x_ = current_x_;
            safe_y_ = current_y_;
            safe_yaw_ = yaw_;
            have_safe_pose_ = true;
        }

        if (debug_mode_)
        {
            auto t = this->now();
            if ((t - last_debug_time_).seconds() >= debug_period_)
            {
                RCLCPP_INFO(this->get_logger(),
                            "pose(x=%.2f y=%.2f yaw=%.2f) | min=%.2f sector=%s | thr=%.2f | safe=%d",
                            current_x_, current_y_, yaw_,
                            min_distance_, sectorToString(obstacle_sector_),
                            threshold_,
                            have_safe_pose_ ? 1 : 0);
                last_debug_time_ = t;

                if (recovery_mode_)
                {
                    RCLCPP_WARN(this->get_logger(), "RECOVERY ENTERED");
                }
            }
        }
    }

    // SUBSCRIBERS
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr user_cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanner_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

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
