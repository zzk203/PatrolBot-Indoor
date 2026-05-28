#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <memory>
#include <atomic>
#include <chrono>

namespace patrol_bot {

enum class NavResult {
    SUCCESS,
    FAILURE,
    RUNNING,
    NOT_STARTED,
    ERROR
};

/// 将 yaw 角度（弧度）转换为 geometry_msgs::msg::Quaternion
geometry_msgs::msg::Quaternion yaw_to_quaternion_msg(double yaw);

class Nav2ActionClient {
public:
    Nav2ActionClient(rclcpp::Node* node,
                     const std::string& action_name = "/navigate_to_pose");

    /// 启用 mock 模式：导航立即返回成功，无需真实 Nav2
    void set_mock(bool mock) { mock_ = mock; }

    bool wait_for_server(std::chrono::seconds timeout);

    void send_goal(double x, double y, double yaw);

    NavResult check_result();

    void cancel_goal();

    bool is_navigating() const { return goal_handle_ != nullptr; }

private:
    rclcpp::Node* node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;

    std::atomic<NavResult> result_{NavResult::NOT_STARTED};
    std::atomic<bool> result_ready_{false};
    std::atomic<bool> server_ready_{false};
    bool mock_ = false;
};

}  // namespace patrol_bot
