#include "action_demo/action/count_down.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <atomic>

class CountDownClient : public rclcpp::Node {
public:
  using CountDown = action_demo::action::CountDown;
  using GoalHandle = rclcpp_action::ClientGoalHandle<CountDown>;

  CountDownClient() : Node("count_down_client"), goal_received_(false) {
    client_ = rclcpp_action::create_client<CountDown>(this, "count_down");
  }

  void send_goal(int target) {
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available, exiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = CountDown::Goal();
    goal_msg.target = target;

    auto options = rclcpp_action::Client<CountDown>::SendGoalOptions();
    options.goal_response_callback = std::bind(
        &CountDownClient::goal_response_callback, this, std::placeholders::_1);
    options.feedback_callback =
        std::bind(&CountDownClient::feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);
    options.result_callback = std::bind(&CountDownClient::result_callback, this,
                                        std::placeholders::_1);

    RCLCPP_INFO(this->get_logger(), "Sending goal: target=%d", target);
    client_->async_send_goal(goal_msg, options);

    double total_timeout_s = static_cast<double>(target) + 5.0;
    timeout_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(total_timeout_s), [this]() {
          RCLCPP_ERROR(this->get_logger(), "Timeout waiting for action result, "
                                           "server may have disconnected");
          rclcpp::shutdown();
        });
  }

private:
  void goal_response_callback(const GoalHandle::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      rclcpp::shutdown();
      return;
    }
    goal_received_ = true;
    goal_handle_ = goal_handle;
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server");

    // cancel_timer_ = this->create_wall_timer(
    //     std::chrono::seconds(3),
    //     [this]() {
    //       RCLCPP_INFO(this->get_logger(), "Requesting cancel...");
    //       client_->async_cancel_goal(goal_handle_);
    //     });
  }

  void
  feedback_callback(GoalHandle::SharedPtr,
                    const std::shared_ptr<const CountDown::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Feedback: current=%d", feedback->current);
  }

  void result_callback(const GoalHandle::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Result: succeeded, final_count=%d",
                  result.result->final_count);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Result: canceled, final_count=%d",
                  result.result->final_count);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Result: aborted");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Result: unknown code=%d",
                   static_cast<int>(result.code));
      break;
    }
    rclcpp::shutdown();
  }

  rclcpp_action::Client<CountDown>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;
  std::atomic<bool> goal_received_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CountDownClient>();
  node->send_goal(5);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
