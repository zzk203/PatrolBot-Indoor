#ifndef NAV2_DEMO__BT_NODES__LOAD_WAYPOINTS_NODE_HPP_
#define NAV2_DEMO__BT_NODES__LOAD_WAYPOINTS_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

#include "nav2_demo/bt_nodes/waypoint_structs.hpp"

namespace nav2_demo
{

  /**
   * @brief M3.2 + M5.1 从 YAML 加载巡逻点的行为树节点
   *
   * 功能:
   *  - 从指定的 YAML 文件读取 patrol_points, charge_standby, charge_dock
   *  - 将数据写入黑板，供后续节点使用
   *  - M5: 额外输出 dock_prep_pose (PoseStamped) 用于 DockActionNode
   *
   * 黑板端口:
   *  - waypoints_file (input, string): YAML 文件路径
   *  - patrol_waypoints (output, vector<Waypoint>): 巡逻点列表
   *  - charge_standby (output, Waypoint): 充电预备点
   *  - charge_dock (output, Waypoint): 充电桩坐标
   *  - dock_prep_pose (output, PoseStamped): 对接预备位姿（由 charge_standby 转换）
   *  - dock_pose_x (output, double): 充电桩 X 坐标
   *  - dock_pose_y (output, double): 充电桩 Y 坐标
   *  - dock_pose_yaw (output, double): 充电桩朝向
   *  - waypoints_count (output, int): 巡逻点数量
   *  - navigation_timeout (output, double): 导航超时时间
   *  - waypoint_wait_duration (output, double): 巡逻点停留时间
   */
class LoadWaypointsNode : public BT::SyncActionNode
{
public:
  LoadWaypointsNode(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

  /// @brief 从 YAML 文件解析 PatrolConfig（可单独测试）
  static PatrolConfig parseYaml(const std::string & yaml_path);
};

}  // namespace nav2_demo

#endif  // NAV2_DEMO__BT_NODES__LOAD_WAYPOINTS_NODE_HPP_
