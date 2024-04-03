#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "example_action_interfaces/action/example.hpp"

class TimerActionClient : public rclcpp::Node {
public:
  using TimerAction = example_action_interfaces::action::Example;
  using GoalHandleTimer = rclcpp_action::ClientGoalHandle<TimerAction>;

  explicit TimerActionClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("timer_action_client", options), goal_done_(false) {
    this->client_ptr_ = rclcpp_action::create_client<TimerAction>(
        this,
        "/timer");

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&TimerActionClient::send_goal, this));
  }

private:
  rclcpp_action::Client<TimerAction>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

  void send_goal() {
    using namespace std::placeholders;
    // Cancel the timer so send_goal is not called again
    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = TimerAction::Goal();
    goal_msg.duration = 10; // Countdown from 10

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<TimerAction>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&TimerActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&TimerActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&TimerActionClient::result_callback, this, _1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(const GoalHandleTimer::SharedPtr future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandleTimer::SharedPtr,
      const std::shared_ptr<const TimerAction::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(),
                "Time remaining: %d",
                feedback->time_remaining);
  }

  void result_callback(const GoalHandleTimer::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded! Time elapsed: %s", result.result->time_elapsed.c_str());
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimerActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}