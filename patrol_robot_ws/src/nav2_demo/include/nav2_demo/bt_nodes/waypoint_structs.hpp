#ifndef NAV2_DEMO__BT_NODES__WAYPOINT_STRUCTS_HPP_
#define NAV2_DEMO__BT_NODES__WAYPOINT_STRUCTS_HPP_

#include <vector>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_demo
{

/// @brief 单个巡逻点数据结构（轻量级，无 ROS 依赖时可独立使用）
struct Waypoint
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;

  /// @brief 转换为 PoseStamped 消息（frame_id="map"）
  geometry_msgs::msg::PoseStamped toPoseStamped() const;
};

/// @brief 从 YAML 加载的完整巡逻配置
struct PatrolConfig
{
  std::vector<Waypoint> patrol_points;
  Waypoint charge_standby;
  Waypoint charge_dock;
  double navigation_timeout = 60.0;
  double waypoint_wait_duration = 3.0;

  bool empty() const { return patrol_points.empty(); }
  size_t size() const { return patrol_points.size(); }
};

}  // namespace nav2_demo

#endif  // NAV2_DEMO__BT_NODES__WAYPOINT_STRUCTS_HPP_
