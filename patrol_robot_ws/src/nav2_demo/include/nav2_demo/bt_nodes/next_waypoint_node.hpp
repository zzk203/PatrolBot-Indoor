#ifndef NAV2_DEMO__BT_NODES__NEXT_WAYPOINT_NODE_HPP_
#define NAV2_DEMO__BT_NODES__NEXT_WAYPOINT_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_demo/bt_nodes/waypoint_structs.hpp"

namespace nav2_demo
{

/**
 * @brief 从黑板 waypoint 列表中取出当前索引对应的巡逻点
 *
 * 功能:
 *  - 从 {patrol_waypoints} 列表中读取 {current_index} 对应的 Waypoint
 *  - 将 Waypoint 转为 PoseStamped 并输出到 {current_waypoint}
 *  - {current_index} 自增 1（当超过 size-1 后不再自增）
 *
 * 返回值:
 *  - SUCCESS: 成功取出下一个巡逻点
 *  - FAILURE: 索引超出范围（巡逻结束）
 *
 * 黑板端口:
 *  - patrol_waypoints (input, vector<Waypoint>): 巡逻点列表
 *  - current_index (input/output, int): 当前索引（调用后自增）
 *  - current_waypoint (output, PoseStamped): 转换后的导航目标
 */
class NextWaypointNode : public BT::SyncActionNode
{
public:
  NextWaypointNode(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace nav2_demo

#endif  // NAV2_DEMO__BT_NODES__NEXT_WAYPOINT_NODE_HPP_
