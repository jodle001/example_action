#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "example_action_interfaces/action/example.hpp"

class TimerActionServer : public rclcpp::Node
{
public:
    using TimerAction = example_action_interfaces::action::Example;
    using GoalHandleTimer = rclcpp_action::ServerGoalHandle<TimerAction>;

    explicit TimerActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("timer_action_server", options)
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<TimerAction>(
            this,
            "/timer",
            std::bind(&TimerActionServer::handle_goal, this, _1, _2),
            std::bind(&TimerActionServer::handle_cancel, this, _1),
            std::bind(&TimerActionServer::handle_accepted, this, _1));
    }

private:
    rclcpp_action::Server<TimerAction>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const TimerAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with duration: %d", goal->duration);
        (void)uuid;
        // Accept all goals
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleTimer> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        // Accept all cancellations
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleTimer> goal_handle)
    {
        using namespace std::placeholders;
        // This needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&TimerActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleTimer> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<TimerAction::Feedback>();
        auto & time_remaining = feedback->time_remaining;

        auto result = std::make_shared<TimerAction::Result>();

        for (int i = goal->duration; i > 0; --i) {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                result->time_elapsed = "Timer canceled";
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            // Publish feedback
            time_remaining = i;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback: %d", time_remaining);

            // Sleep for one second
            rclcpp::sleep_for(std::chrono::seconds(1));
        }

        // Goal completed
        result->time_elapsed = "Timer completed";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimerActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}