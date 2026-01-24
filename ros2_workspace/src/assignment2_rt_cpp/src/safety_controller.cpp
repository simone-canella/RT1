#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include "assignment2_rt_cpp/msg/obstacle_info.hpp"

#include <chrono>
#include <limits>
#include <cmath>
#include <algorithm>
#include <iostream>

class SafetyController : public rclcpp::Node
{
public:
    SafetyController() : Node("safety_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Safety Controller started");

        // DEFAULT PARAMETERS
        // threshold: minimum allowed distance to obstacles (meters)
        this->declare_parameter<double>("threshold", 0.8);
        threshold_ = this->get_parameter("threshold").as_double();

        // debug: enable periodic logs; debug_period: seconds between logs
        this->declare_parameter<bool>("debug", false);
        this->declare_parameter<double>("debug_period", 1.0);
        debug_mode_ = this->get_parameter("debug").as_bool();
        debug_period_ = this->get_parameter("debug_period").as_double();
        last_debug_time_ = this->now();

        // SUBSCRIBERS
        // subscribe to user command: user velocity commands (from teleop_interface node)
        user_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/user_cmd_vel", 10,
            std::bind(&SafetyController::userCommandCallback, this, std::placeholders::_1));

        // subscribe to laser scan: 360 scan
        scanner_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SafetyController::scannerCallback, this, std::placeholders::_1));

        // subscribe to odometry: position + orientation quaternion
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&SafetyController::odomCallback, this, std::placeholders::_1));

        // PUBLISHERS
        // publisher to cmd_vel
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "cmd_vel publisher created");

        // publisher to custom message
        obstacle_pub_ = this->create_publisher<assignment2_rt_cpp::msg::ObstacleInfo>("/obstacle_info", 10);

        // TIMER
        // run the "control loop"
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // publish rate = 50Hz (20mS)
            std::bind(&SafetyController::controlLoop, this));
    }

private:
    // DEBUG FEATURES
    bool debug_mode_ = false;
    double debug_period_ = 1.0;
    rclcpp::Time last_debug_time_;

    // helper function for debugging
    bool shouldPrintDebug()
    {
        if (!debug_mode_)
            return false;

        auto now = this->now();
        if ((now - last_debug_time_).seconds() >= debug_period_)
        {
            last_debug_time_ = now;
            return true;
        }
        return false;
    }

    // VARIABLES + CONTROLS STATE
    // variable to store the last command
    geometry_msgs::msg::Twist last_user_cmd_;

    bool have_user_cmd_ = false; // control if the user input is arrived

    float min_distance_ = std::numeric_limits<float>::infinity(); // distance of closest obstacle
    uint8_t obstacle_sector_ = 0;                                 // 0=FRONT, 1=LEFT, 2=BACK, 3=RIGHT

    // current position and orientation
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double yaw_ = 0.0;       // robot orientation
    bool have_odom_ = false; // control if the odom is arrived

    // recovery parameters
    double threshold_ = 0.8; // default (meters)

    // safe position
    double safe_x_ = 0.0;
    double safe_y_ = 0.0;
    double safe_yaw_ = 0.0;
    bool have_safe_pose_ = false; // control if the safe pose is arrived

    bool recovery_mode_ = false; // control if the recovery mode is activated
    bool is_moving_cmd_ = false; // control if the robot is moving

    bool cmd_active_ = false;
    bool recovery_happened_in_cmd_ = false;
    bool pending_safe_update_ = false;

    double k_lin_ = 0.8;                // linear gain
    double k_yaw_ = 1.5;                // angular gain
    double max_lin_ = 0.3;              // m/s clamp
    double max_yaw_ = 1.0;              // rad/s clamp
    double position_tollerance_ = 0.10; // meters
    double yaw_tollerance_ = 0.30;      // radians

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

    // given an angle, select in which sector it belongs
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

    // clamp helper
    static double clamp(double v, double lo, double hi)
    {
        return std::max(lo, std::min(v, hi));
    }

    // CALLBACKS
    // user command callback:
    void userCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_user_cmd_ = *msg;
        have_user_cmd_ = true;

        // set that the robot is moving
        const bool moving = (std::fabs(msg->linear.x) > 1e-3) || (std::fabs(msg->angular.z) > 1e-3);
        is_moving_cmd_ = moving;

        if (moving)
        {
            cmd_active_ = true;
        }
        else
        {
            // command ended (teleop sends zero)
            if (cmd_active_ && !recovery_happened_in_cmd_)
            {
                pending_safe_update_ = true; // update safe pose once we are stopped in NORMAL mode
            }
            cmd_active_ = false;
            recovery_happened_in_cmd_ = false;
        }

        if (debug_mode_)
        {
            RCLCPP_INFO(this->get_logger(), "Got user cmd: lin.x=%.2f ang.z=%.2f",
                        msg->linear.x, msg->angular.z);
        }
    }

    // scanner callback: compute closest obstacle distance and its sector.
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
        yaw_ = tf2::getYaw(msg->pose.pose.orientation);

        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        have_odom_ = true;

        // initialize safe pose to startup pose (first odomometry received).
        if (!have_safe_pose_)
        {
            safe_x_ = current_x_;
            safe_y_ = current_y_;
            safe_yaw_ = yaw_;
            have_safe_pose_ = true;
        }
    }

    // timer callback: where decisions are taken
    void controlLoop()
    {
        // SAFE POSE UPDATE
        // apply safe-pose update only after a safe user command ended
        if (!recovery_mode_ && pending_safe_update_ && have_odom_)
        {
            safe_x_ = current_x_;
            safe_y_ = current_y_;
            safe_yaw_ = yaw_;
            have_safe_pose_ = true;
            pending_safe_update_ = false;

            if (debug_mode_)
            {
                RCLCPP_WARN(this->get_logger(), "SAFE POSE UPDATED to x=%.2f y=%.2f", safe_x_, safe_y_);
            }
        }

        // RECOVERY MODE
        // enter in recovery mode if robot is too close to an obstacle during a command burst.
        if (!recovery_mode_ && have_safe_pose_ && have_user_cmd_ && is_moving_cmd_ && std::isfinite(min_distance_) && min_distance_ <= threshold_)
        {
            recovery_mode_ = true;
            recovery_happened_in_cmd_ = true;
            pending_safe_update_ = false;

            RCLCPP_WARN(this->get_logger(), "RECOVERY ENTERED");

            // stop accepting the current user command
            have_user_cmd_ = false;
        }

        // recovery procedure: go back to the safe pose
        if (recovery_mode_)
        {
            // difference between safe position and current position
            double difference_x = safe_x_ - current_x_;
            double difference_y = safe_y_ - current_y_;
            double safe_distance = std::hypot(difference_x, difference_y);

            double target_yaw = std::atan2(difference_y, difference_x);
            double yaw_error = normalizeAngle(target_yaw - yaw_);

            // control if the robot is in safe position
            if (safe_distance < position_tollerance_)
            {
                cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
                recovery_mode_ = false;
                have_user_cmd_ = false;
                is_moving_cmd_ = false;
                return;
            }

            // return to safe position
            geometry_msgs::msg::Twist cmd;

            if (std::fabs(yaw_error) > yaw_tollerance_)
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = clamp(k_yaw_ * yaw_error, -max_yaw_, max_yaw_);
            }
            else
            {
                cmd.linear.x = clamp(k_lin_ * safe_distance, 0.0, max_lin_);
                cmd.angular.z = clamp(k_yaw_ * yaw_error, -max_yaw_, max_yaw_);
            }

            if (shouldPrintDebug())
            {
                RCLCPP_INFO(this->get_logger(),
                            "RECOVERY: dist=%.2f yaw_err=%.2f | min=%.2f sector=%s",
                            safe_distance, yaw_error, min_distance_, sectorToString(obstacle_sector_));
            }

            cmd_vel_pub_->publish(cmd);
            return;
        }

        // NORMAL MODE
        // publish user command
        if (have_user_cmd_)
        {
            cmd_vel_pub_->publish(last_user_cmd_);
        }
        else
        {
            geometry_msgs::msg::Twist stop;
            cmd_vel_pub_->publish(stop);
        }

        // publish obstacle information with the custom message "ObstacleInfo"
        assignment2_rt_cpp::msg::ObstacleInfo info;
        info.closest_distance = static_cast<float>(min_distance_);
        info.direction = obstacle_sector_;
        info.threshold = static_cast<float>(threshold_);
        obstacle_pub_->publish(info);

        // DEBUGGING
        // print log for debugging
        if (shouldPrintDebug())
        {
            RCLCPP_INFO(this->get_logger(),
                        "pose(x=%.2f y=%.2f yaw=%.2f) | min=%.2f sector=%s | thr=%.2f | safe=%d",
                        current_x_, current_y_, yaw_,
                        min_distance_, sectorToString(obstacle_sector_),
                        threshold_,
                        have_safe_pose_ ? 1 : 0);
        }
    }

    // SUBSCRIBERS INTERFACES
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr user_cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanner_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    // PUBLISHERS INTERFACE
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<assignment2_rt_cpp::msg::ObstacleInfo>::SharedPtr obstacle_pub_;

    // TIMER INTERFACES
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
