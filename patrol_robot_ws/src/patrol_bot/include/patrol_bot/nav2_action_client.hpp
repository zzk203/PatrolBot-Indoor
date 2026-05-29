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
    SUCCESS,      // 导航成功到达
    FAILURE,      // 导航失败 (Nav2 返回 ABORTED 或 CANCELED)
    RUNNING,      // 导航进行中
    NOT_STARTED,  // 尚未发送 goal
    ERROR         // Action 通信异常 (服务端未就绪等)
};

/// 将 yaw 角度（弧度）转换为 geometry_msgs::msg::Quaternion
geometry_msgs::msg::Quaternion yaw_to_quaternion_msg(double yaw);

class Nav2ActionClient {
public:
    Nav2ActionClient(rclcpp::Node* node, const bool mock = false,
                     const std::string& action_name = "/navigate_to_pose");

    /// 启用 mock 模式：导航立即返回成功，无需真实 Nav2
    void set_mock(bool mock);

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
