#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "assignment1_rt2_interfaces/action/robot_target.hpp"

#include "rclcpp_components/register_node_macro.hpp"

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
            : Node("robot_action_client", options), goal_done_(true)
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

        // atomic thread
        std::atomic<bool> goal_done_;

        // CALLBACK
        // goal response callback: control if the goal is accepted
        void goal_response_callback(const GoalHandleRobotTarget::SharedPtr &goal_handle)
        {
            if (!goal_handle)
            {
                // release the UI thread if rejected!
                goal_done_ = true;
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
            // mark that the goal is reached
            this->goal_done_ = true;

            // switch for knowing the final server action result
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
                // wait if the robot is busy
                while (!goal_done_)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

                // ask the user if the robot is idle
                double x, y, theta;

                std::cout << "Enter coordinates (X Y Theta) separated by spaces: ";

                // input chained verification
                if (!(std::cin >> x >> y >> theta))
                {
                    std::cout << "Invalid input! Please enter three numbers (e.g., 2.0 1.5 0.0)." << std::endl;
                    std::cin.clear();
                    std::cin.ignore(1000, '\n');
                    continue; // restart the prompt
                }

                RCLCPP_INFO(this->get_logger(), "User input received: x= %f, y=%f, theta=%f", x, y, theta);

                // mark the robot as busy
                goal_done_ = false;

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

// macro for recognize that is a loadable component
RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_rt2::RobotActionClient)

// entry point to start the node
int main(int argc, char **argv)
{
    // initialize ROS 2 communication
    rclcpp::init(argc, argv);

    // initialize the client node
    auto node = std::make_shared<assignment1_rt2::RobotActionClient>(rclcpp::NodeOptions());

    // keeps the node alive and processing ROS messages
    rclcpp::spin(node);

    // shutdown
    rclcpp::shutdown();
    return 0;
}