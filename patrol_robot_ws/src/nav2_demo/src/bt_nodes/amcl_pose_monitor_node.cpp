#include "nav2_demo/bt_nodes/amcl_pose_monitor_node.hpp"

#include <string>
#include <memory>
#include <algorithm>
#include <cmath>

namespace nav2_demo
{

AmclPoseMonitorNode::AmclPoseMonitorNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::ConditionNode(name, config)
, node_(node)
{
  // 初始化协方差数组
  latest_covariance_diag_.fill(0.0);

  // 订阅 /amcl_pose 话题
  amcl_pose_sub_ =
    node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose",
    rclcpp::QoS(10).transient_local(),
    std::bind(&AmclPoseMonitorNode::amclPoseCallback, this, std::placeholders::_1));

  // 创建 /patrol_alerts 发布者
  alert_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/patrol_alerts", rclcpp::QoS(10).transient_local());

  RCLCPP_DEBUG(node_->get_logger(), "AmclPoseMonitorNode: 已订阅 /amcl_pose");
}

BT::PortsList AmclPoseMonitorNode::providedPorts()
{
  return {
    BT::InputPort<double>("covariance_threshold", DEFAULT_COVARIANCE_THRESHOLD,
      "AMCL covariance threshold for localization quality"),
    BT::OutputPort<bool>("localization_valid", "Localization validity flag"),
  };
}

BT::NodeStatus AmclPoseMonitorNode::tick()
{
  double threshold = getInput<double>("covariance_threshold")
    .value_or(DEFAULT_COVARIANCE_THRESHOLD);

  // 读取最新协方差
  double max_cov = 0.0;
  bool has_data = false;
  {
    std::lock_guard<std::mutex> lock(cov_mutex_);
    if (has_covariance_) {
      // 取协方差矩阵对角线的最大值作为定位质量指标
      // 协方差矩阵为 6x6，对角元为 [x, y, z, roll, pitch, yaw] 的方差
      // 我们关注 x, y, yaw 的方差（位置和朝向）
      max_cov = std::max({latest_covariance_diag_[0],  // x
                          latest_covariance_diag_[1],  // y
                          latest_covariance_diag_[5]}); // yaw
      has_data = true;
    }
  }

  if (!has_data) {
    // 尚未收到 AMCL 数据：保守策略，认为定位无效
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "AmclPoseMonitorNode: 尚未收到 AMCL 位姿数据");
    setOutput("localization_valid", false);
    return BT::NodeStatus::FAILURE;
  }

  // 指数平滑，防止瞬态抖动导致误报
  smoothed_cov_ = SMOOTHING_ALPHA * max_cov + (1.0 - SMOOTHING_ALPHA) * smoothed_cov_;

  if (smoothed_cov_ > threshold) {
    // 定位质量差，发布告警
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
      "AmclPoseMonitorNode: 定位质量下降! 平滑协方差=%.4f (阈值=%.2f)",
      smoothed_cov_, threshold);

    publishAlert("Localization quality degraded (covariance=" +
      std::to_string(smoothed_cov_) + ")");

    setOutput("localization_valid", false);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_DEBUG(node_->get_logger(),
    "AmclPoseMonitorNode: 定位正常 cov=%.4f threshold=%.2f",
    smoothed_cov_, threshold);

  setOutput("localization_valid", true);
  return BT::NodeStatus::SUCCESS;
}

void AmclPoseMonitorNode::amclPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cov_mutex_);

  // 提取协方差矩阵对角元素（6x6 矩阵存储为 36 元素的一维数组，行主序）
  // 对角元索引: 0, 7, 14, 21, 28, 35
  const auto & cov = msg->pose.covariance;
  if (cov.size() >= 36) {
    latest_covariance_diag_[0] = cov[0];   // x
    latest_covariance_diag_[1] = cov[7];   // y
    latest_covariance_diag_[2] = cov[14];  // z
    latest_covariance_diag_[3] = cov[21];  // roll
    latest_covariance_diag_[4] = cov[28];  // pitch
    latest_covariance_diag_[5] = cov[35];  // yaw
    has_covariance_ = true;
  }
}

void AmclPoseMonitorNode::publishAlert(const std::string & msg)
{
  auto alert_msg = std::make_unique<std_msgs::msg::String>();
  alert_msg->data = msg;
  alert_pub_->publish(std::move(alert_msg));
}

}  // namespace nav2_demo
