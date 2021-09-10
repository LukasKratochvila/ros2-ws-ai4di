#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "custom_interfaces/action/custom_action.hpp"
#include "cpp_examples/visibility_control.h"


class CountingActionServer : public rclcpp::Node
{
public:

    using CustomAction = custom_interfaces::action::CustomAction;
    using GoalHandleCustomAction = rclcpp_action::ServerGoalHandle<CustomAction>;

    ACTION_TUTORIALS_CPP_PUBLIC

    explicit CountingActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
            : Node("counting_action_server", options)
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<CustomAction>(
                this,
                "counting",
                std::bind(&CountingActionServer::handle_goal, this, _1, _2),
                std::bind(&CountingActionServer::handle_cancel, this, _1),
                std::bind(&CountingActionServer::handle_accepted, this, _1));
    }

private:
    rclcpp_action::Server<CustomAction>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const CustomAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->count_to_goal);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleCustomAction> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleCustomAction> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&CountingActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleCustomAction> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<CustomAction::Feedback>();
        feedback->feedback_state = 0;
        auto result = std::make_shared<CustomAction::Result>();

        for (int i = 1; (i < goal->count_to_goal) && rclcpp::ok(); i++) {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                result->result_count = feedback->feedback_state;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            // Update sequence
            feedback->feedback_state = i;
            // Publish feedback
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback");

            loop_rate.sleep();
        }

        // Check if goal is done
        if (rclcpp::ok()) {
            result->result_count = feedback->feedback_state + 1;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<CountingActionServer>();
    rclcpp::spin(action_server);
    return 0;
}