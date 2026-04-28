#include <iostream>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "assignment1_rt2_interfaces/action/robot_target.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

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
        //publisher
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
            RCLCPP_INFO(this->get_logger(), "Executing goal");
            auto result = std::make_shared<RobotTarget::Result>();

            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
        }
    };
}

// MACRO
RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_rt2::RobotActionServer)