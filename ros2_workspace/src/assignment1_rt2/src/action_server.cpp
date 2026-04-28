#include <iostream>
#include <thread>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "assignment1_rt2_interfaces/action/robot_target.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/utils.h>
#include <tf2/exceptions.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define THRESHOLD 0.1    // distance threshold between current and goal positions
#define LINEAR_GAIN 0.5  // linear velocity gain for proportional logic controller
#define ANGULAR_GAIN 1.0 // angular velocity gain for proportional logic controller

namespace assignment1_rt2
{
    class RobotActionServer : public rclcpp::Node
    {
    public:
        // ALIAS
        using RobotTarget = assignment1_rt2_interfaces::action::RobotTarget;
        using GoalHandleRobotTarget = rclcpp_action::ServerGoalHandle<RobotTarget>;

        // CONSTRUCTOR
        explicit RobotActionServer(const rclcpp::NodeOptions &options)
            : Node("robot_action_server", options)
        {
            using namespace std::placeholders;

            // TF2 LOCALIZATION SYSTEM
            // initialize tf2 BUFFER and LISTENER
            tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
            RCLCPP_INFO(this->get_logger(), "TF2 Senses initialized.");

            // PUBLISHER
            // initialize publisher to cmd_vel
            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            RCLCPP_INFO(this->get_logger(), "cmd_vel publisher created");

            // initialize the action server
            this->action_server_ = rclcpp_action::create_server<RobotTarget>(
                this,
                "robot_target",                                            // name of the action topic
                std::bind(&RobotActionServer::handle_goal, this, _1, _2),  // goal callback
                std::bind(&RobotActionServer::handle_cancel, this, _1),    // cancel callback
                std::bind(&RobotActionServer::handle_accepted, this, _1)); // accepted callback
            RCLCPP_INFO(this->get_logger(), "Action Server Component is ready!");
        }

    private:
        // OBJECTS
        // create the action server object
        rclcpp_action::Server<RobotTarget>::SharedPtr action_server_;
        // publisher
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

        // TF2 LOCALIZATION SYSTEM
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

        // CALLBACK
        // Handle receiving a goal
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const RobotTarget::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request with x: %f, y: %f", goal->x, goal->y);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        // Handle cancellation
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleRobotTarget> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // Handle what happens after acceptance
        void handle_accepted(const std::shared_ptr<GoalHandleRobotTarget> goal_handle)
        {
            // This needs to return quickly, so we usually spin up a thread
            // to do the actual driving logic!
            std::thread{std::bind(&RobotActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
        }

        // PROCESSING AND UPDATES FUNCTION
        void execute(const std::shared_ptr<GoalHandleRobotTarget> goal_handle)
        {
            // declare variables
            double current_x = 0.0;
            double current_y = 0.0;
            double current_yaw = 0.0;
            double distance_to_goal = 0.0;
            double desired_yaw = 0.0;
            double yaw_error = 0.0;

            RCLCPP_INFO(this->get_logger(), "Executing goal...");

            // get the goal data from the goal_handle
            const auto goal = goal_handle->get_goal();

            // create an instance for "feedback" and "result"
            auto feedback = std::make_shared<RobotTarget::Feedback>();
            auto result = std::make_shared<RobotTarget::Result>();

            // define the frequency rate
            rclcpp::Rate loop_rate(10);

            // navigation loop
            while (rclcpp::ok())
            {
                // check if the action is cancelled
                if (goal_handle->is_canceling())
                {
                    result->success = false;
                    goal_handle->canceled(result);
                    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});

                    return;
                }

                // declare message variable
                geometry_msgs::msg::TransformStamped transform;

                // position tracking block
                try
                {
                    transform = tf2_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

                    // extract current position and orientation
                    current_x = transform.transform.translation.x;
                    current_y = transform.transform.translation.y;

                    current_yaw = tf2::getYaw(transform.transform.rotation);

                    // compute distance between robot and goal
                    double difference_x = goal->x - current_x;
                    double difference_y = goal->y - current_y;

                    distance_to_goal = std::hypot(difference_x, difference_y);

                    // update feedback
                    feedback->dist_x = difference_x;
                    feedback->dist_y = difference_y;

                    goal_handle->publish_feedback(feedback);
                    RCLCPP_INFO(this->get_logger(), "Distance to goal: %f", distance_to_goal);

                    // compute desired heading and heading error
                    desired_yaw = std::atan2(difference_y, difference_x);

                    yaw_error = normalizeAngle(desired_yaw - current_yaw);

                    // create command message
                    auto move_msg = geometry_msgs::msg::Twist();

                    // proportional control logic
                    move_msg.linear.x = LINEAR_GAIN * distance_to_goal;
                    move_msg.angular.z = ANGULAR_GAIN * yaw_error;

                    // send message
                    cmd_vel_pub_->publish(move_msg);

                    // stop when the goal is reached
                    if (distance_to_goal < THRESHOLD)
                    {
                        cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});

                        RCLCPP_INFO(this->get_logger(), "Goal reached");
                        break;
                    }
                }
                catch (const tf2::TransformException &ex)
                {
                    RCLCPP_INFO(this->get_logger(), "Could not transform map to base_link: %s", ex.what());

                    // wait for the next loop
                    loop_rate.sleep();
                    continue;
                }

                loop_rate.sleep();
            }

            if (rclcpp::ok())
            {
                result->success = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
            }
        }

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
    };
}

// MACRO
RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_rt2::RobotActionServer)