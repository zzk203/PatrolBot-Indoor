#ifndef NAV2_DEMO__NAV2_DEMO_NODE_HPP_
#define NAV2_DEMO__NAV2_DEMO_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace nav2_demo
{

/**
 * @brief Nav2 导航演示节点
 *
 * 封装了 navigate_to_pose 动作客户端，支持：
 * - 从参数或代码配置多个航点
 * - 顺序发送导航目标
 * - 反馈回调（剩余距离、预计时间）
 * - 取消当前目标
 * - 结果回调（成功/取消/失败）
 */
class Nav2DemoNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  /**
   * @brief 构造函数
   * @param options 节点选项
   */
  explicit Nav2DemoNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /**
   * @brief 发送下一个航点目标
   *
   * 如果所有航点已完成，则打印完成信息。
   */
  void send_next_goal();

  /**
   * @brief 取消当前导航目标
   */
  void cancel_current_goal();

  /**
   * @brief 获取航点总数
   */
  size_t get_goal_count() const { return goals_.size(); }

  /**
   * @brief 获取当前航点索引
   */
  size_t get_current_goal_index() const { return goal_index_; }

  /**
   * @brief 检查是否所有航点已完成
   */
  bool all_goals_completed() const { return goal_index_ >= goals_.size(); }

  /**
   * @brief 设置航点列表（覆盖默认）
   * @param goals 航点列表，每个航点为 [x, y, yaw]
   */
  void set_goals(const std::vector<std::array<double, 3>> &goals)
  {
    goals_ = goals;
    goal_index_ = 0;
  }

  /**
   * @brief 检查动作服务端是否就绪
   */
  bool is_action_server_ready(const std::chrono::seconds &timeout = std::chrono::seconds(5));

private:
  // === 回调函数 ===

  /**
   * @brief 全局路径回调
   */
  void plan_callback(const nav_msgs::msg::Path::SharedPtr msg);

  /**
   * @brief 里程计回调
   */
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief 导航结果回调
   */
  void result_callback(const GoalHandleNav::WrappedResult &result);

  /**
   * @brief 导航反馈回调
   */
  void feedback_callback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);

  /**
   * @brief 计算路径总长度
   */
  static double calculate_path_length(const nav_msgs::msg::Path &path);

  // === 声明参数 ===
  void declare_parameters();

  // === 从参数加载航点 ===
  void load_goals_from_parameters();

  // === 成员变量 ===
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // 当前发送的目标句柄（用于取消）
  std::shared_future<GoalHandleNav::SharedPtr> goal_handle_future_;

  // 航点列表（每个元素为 [x, y, yaw]）
  std::vector<std::array<double, 3>> goals_;
  size_t goal_index_;

  // 当前导航状态
  std::atomic<bool> goal_active_{false};
};

}  // namespace nav2_demo

#endif  // NAV2_DEMO__NAV2_DEMO_NODE_HPP_
