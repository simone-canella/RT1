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
        }

    private:
        // OBJECTS
        // create the action client object
        rclcpp_action::Client<RobotTarget>::SharedPtr client_ptr_;

        void ui_loop() {
            
        }
    };
}