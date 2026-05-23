#include "action_demo/action/count_down.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class CountDownServer : public rclcpp::Node {
public:
  using CountDown = action_demo::action::CountDown;
  using GoalHandle = rclcpp_action::ServerGoalHandle<CountDown>;

  CountDownServer() : Node("count_down_server") {
    action_server_ = rclcpp_action::create_server<CountDown>(
        this, "count_down",
        std::bind(&CountDownServer::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&CountDownServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&CountDownServer::handle_accepted, this,
                  std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "CountDown action server ready");
  }

private:
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &,
              std::shared_ptr<const CountDown::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal: target=%d", goal->target);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread(std::bind(&CountDownServer::execute, this, goal_handle))
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CountDown::Feedback>();
    auto result = std::make_shared<CountDown::Result>();

    for (int i = goal->target; i >= 0; i--) {
      if (goal_handle->is_canceling()) {
        result->final_count = i;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled at count=%d", i);
        return;
      }
      feedback->current = i;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publishing feedback: current=%d", i);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    result->final_count = 0;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }

  rclcpp_action::Server<CountDown>::SharedPtr action_server_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CountDownServer>());
  rclcpp::shutdown();
  return 0;
}
