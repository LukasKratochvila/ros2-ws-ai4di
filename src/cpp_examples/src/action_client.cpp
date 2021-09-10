#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "custom_interfaces/action/custom_action.hpp"

class CountingActionClient : public rclcpp::Node
{
public:

    using CustomAction = custom_interfaces::action::CustomAction;
    using GoalHandleCustomAction = rclcpp_action::ClientGoalHandle<CustomAction>;

    explicit CountingActionClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
            : Node("counting_action_client", options)
    {
        this->client_ptr_ = rclcpp_action::create_client<CustomAction>(
                this,
                "counting");

        this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&CountingActionClient::send_goal, this));
    }

    void send_goal()
    {
        using namespace std::placeholders;
        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = CustomAction::Goal();
        goal_msg.count_to_goal = 10;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<CustomAction>::SendGoalOptions();
        send_goal_options.goal_response_callback =
                std::bind(&CountingActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
                std::bind(&CountingActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
                std::bind(&CountingActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<CustomAction>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_response_callback(std::shared_future<GoalHandleCustomAction::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleCustomAction::SharedPtr, const std::shared_ptr<const CustomAction::Feedback> feedback) {
        std::stringstream ss;
        ss << "Feedback: " << feedback->feedback_state;
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    void result_callback(const GoalHandleCustomAction::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
        std::stringstream ss;
        ss << "Result received: " << result.result->result_count;
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        rclcpp::shutdown();
    }
};


int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<CountingActionClient>();
    rclcpp::spin(action_server);
    return 0;
}