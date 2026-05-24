#include "nav2_demo/bt_nodes/waypoint_structs.hpp"

#include <cmath>

namespace nav2_demo
{

geometry_msgs::msg::PoseStamped Waypoint::toPoseStamped() const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  // yaw → 四元数（仅绕 Z 轴旋转）
  pose.pose.orientation.z = std::sin(yaw / 2.0);
  pose.pose.orientation.w = std::cos(yaw / 2.0);
  // x, y 分量为 0（纯绕 Z 轴）
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;

  return pose;
}

}  // namespace nav2_demo
