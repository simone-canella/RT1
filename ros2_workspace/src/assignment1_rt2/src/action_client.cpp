#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "assignment1_rt2_interfaces/action/robot_target.hpp"

namespace assignment1_rt2
{
    class RobotActionClient : public rclcpp::Node
    {
    public:
        // ALIAS
        using RobotTarget = assignment1_rt2_interfaces::action::RobotTarget;
        using GoalHandleRobotTarget = rclcpp_action::ClientGoalHandle<RobotTarget>;

        // CONSTRUCTOR
        explicit RobotActionClient(const rclcpp::NodeOptions &options)
            : Node("robot_action_client", options)
        {
            // initialize the action client
            this->client_ptr_ = rclcpp_action::create_client<RobotTarget>(
                this,
                "robot_target");

            // check if the action server is alive
            if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10)))
            {
                RCLCPP_ERROR(this->get_logger(), "Action server not available!");
                return;
            }

            // launch a new thread for UI loop (avoiding block execution)
            std::thread{std::bind(&RobotActionClient::ui_loop, this)}.detach();
        }

    private:
        // OBJECTS
        // create the action client object
        rclcpp_action::Client<RobotTarget>::SharedPtr client_ptr_;

        // CALLBACK
        // goal response callback: control if the goal is accepted
        void goal_response_callback(const GoalHandleRobotTarget::SharedPtr &goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_INFO(this->get_logger(), "Goal was rejected by the server");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by the server, waiting for result...");
            }
        }

        // feedback callback: control action server feedback
        void feedback_callback(GoalHandleRobotTarget::SharedPtr,
                               const std::shared_ptr<const RobotTarget::Feedback> feedback)
        {
            RCLCPP_INFO(this->get_logger(), "Distance to goal: x=%f, y=%f", feedback->dist_x, feedback->dist_y);
        }

        // result callback: control the final result
        void result_callback(const GoalHandleRobotTarget::WrappedResult &result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Mission Complete: Goal reached successfully!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Mission Failed: The goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Mission Canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code received");
                break;
            }
        }

        // INTERNAL FUNCTIONS
        // loop for user's inputs
        void ui_loop()
        {
            while (rclcpp::ok())
            {
                double x, y, theta;

                std::cout << "\n ---- TARGET INPUT ----" << std::endl;
                std::cout << "Enter X coordinate: ";
                std::cin >> x;
                std::cout << "Enter Y coordinate: ";
                std::cin >> y;
                std::cout << "Enter the orientation coordinate: ";
                std::cin >> theta;

                RCLCPP_INFO(this->get_logger(), "User input received: x= %f, y=%f, theta=%f", x, y, theta);

                // call the function to send the goal
                this->send_goal(x, y, theta);
            }
        }

        // send the goal position to the action server
        void send_goal(double x, double y, double theta)
        {
            RCLCPP_INFO(this->get_logger(), "Sending goal coordinates to the server...");

            // create goal message
            auto goal_msg = RobotTarget::Goal();

            goal_msg.x = x;
            goal_msg.y = y;
            goal_msg.theta = theta;

            // create a setting for the goal
            auto send_goal_options = rclcpp_action::Client<RobotTarget>::SendGoalOptions();

            // link the goal response callback with the setting
            send_goal_options.goal_response_callback =
                std::bind(&RobotActionClient::goal_response_callback, this, std::placeholders::_1);

            // link the feedback callback with the options
            send_goal_options.feedback_callback =
                std::bind(&RobotActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

            // link the result callback with the options
            send_goal_options.result_callback =
                std::bind(&RobotActionClient::result_callback, this, std::placeholders::_1);

            // send message
            this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        }
    };
}