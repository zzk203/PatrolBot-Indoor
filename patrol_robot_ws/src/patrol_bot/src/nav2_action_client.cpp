#include "patrol_bot/nav2_action_client.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace patrol_bot {

geometry_msgs::msg::Quaternion yaw_to_quaternion_msg(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    geometry_msgs::msg::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    return msg;
}

Nav2ActionClient::Nav2ActionClient(rclcpp::Node* node, const std::string& action_name)
    : node_(node) {
    client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, action_name);
}

bool Nav2ActionClient::wait_for_server(std::chrono::seconds timeout) {
    server_ready_ = client_->wait_for_action_server(timeout);
    return server_ready_.load();
}

void Nav2ActionClient::send_goal(double x, double y, double yaw) {
    if (!server_ready_) {
        result_ = NavResult::ERROR;
        result_ready_ = true;
        return;
    }

    cancel_goal();

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = node_->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation = yaw_to_quaternion_msg(yaw);

    using GoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this](const GoalHandle::SharedPtr& response) {
            if (!response) {
                result_ = NavResult::ERROR;
                result_ready_ = true;
                return;
            }
            goal_handle_ = response;
        };

    send_goal_options.result_callback =
        [this](const GoalHandle::WrappedResult& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                result_ = NavResult::SUCCESS;
            } else {
                result_ = NavResult::FAILURE;
            }
            result_ready_ = true;
            goal_handle_.reset();
        };

    client_->async_send_goal(goal_msg, send_goal_options);
}

NavResult Nav2ActionClient::check_result() {
    if (result_ready_) {
        return result_;
    }
    if (goal_handle_ != nullptr) {
        return NavResult::RUNNING;
    }
    return NavResult::NOT_STARTED;
}

void Nav2ActionClient::cancel_goal() {
    if (goal_handle_ != nullptr) {
        client_->async_cancel_goal(goal_handle_);
    }
    goal_handle_.reset();
    result_ = NavResult::NOT_STARTED;
    result_ready_ = false;
}

}  // namespace patrol_bot
