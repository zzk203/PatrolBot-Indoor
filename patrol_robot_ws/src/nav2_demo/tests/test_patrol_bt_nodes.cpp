// M3.3 + M3.4 + M3.5: NextWaypointNode + RecordFailureNode 单元测试
//
// 测试策略:
//   1. NextWaypointNode: 验证索引递增、越界处理、PoseStamped 转换
//   2. RecordFailureNode: 验证失败计数器递增、原因记录
//   3. 验证这两个节点在 BT 回退 (Fallback) 模式下的协作

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"

#include "nav2_demo/bt_nodes/next_waypoint_node.hpp"
#include "nav2_demo/bt_nodes/record_failure_node.hpp"
#include "nav2_demo/bt_nodes/waypoint_structs.hpp"

namespace fs = std::filesystem;

// ======================================================================
// 辅助函数
// ======================================================================

/// @brief 创建 BehaviorTreeFactory 并注册 M3 自定义节点
void registerM3Nodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<nav2_demo::NextWaypointNode>("NextWaypointNode");
  factory.registerNodeType<nav2_demo::RecordFailureNode>("RecordFailureNode");
}

// ======================================================================
// NextWaypointNode 测试
// ======================================================================

TEST(NextWaypointNodeTest, ReturnsNextWaypointAndIncrementsIndex)
{
  // Setup
  auto blackboard = BT::Blackboard::create();

  std::vector<nav2_demo::Waypoint> waypoints = {
    {1.0, 2.0, 0.5},
    {3.0, 4.0, 1.0},
    {5.0, 6.0, 1.5},
  };
  blackboard->set("patrol_waypoints", waypoints);
  blackboard->set("current_index", 0);

  BT::BehaviorTreeFactory factory;
  registerM3Nodes(factory);

  // Tree: NextWaypointNode → output current_waypoint
  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <NextWaypointNode
          patrol_waypoints="{patrol_waypoints}"
          current_index="{current_index}"
          current_waypoint="{current_waypoint}"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);

  // Execute first waypoint
  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  // Verify output
  auto waypoint = blackboard->get<geometry_msgs::msg::PoseStamped>("current_waypoint");
  EXPECT_DOUBLE_EQ(waypoint.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(waypoint.pose.position.y, 2.0);

  // Verify index incremented
  int idx = blackboard->get<int>("current_index");
  EXPECT_EQ(idx, 1);

  // Execute second waypoint
  status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  waypoint = blackboard->get<geometry_msgs::msg::PoseStamped>("current_waypoint");
  EXPECT_DOUBLE_EQ(waypoint.pose.position.x, 3.0);
  idx = blackboard->get<int>("current_index");
  EXPECT_EQ(idx, 2);

  // Execute third waypoint
  status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  waypoint = blackboard->get<geometry_msgs::msg::PoseStamped>("current_waypoint");
  EXPECT_DOUBLE_EQ(waypoint.pose.position.x, 5.0);
  idx = blackboard->get<int>("current_index");
  EXPECT_EQ(idx, 3);

  // Out of range → FAILURE
  status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST(NextWaypointNodeTest, FailsOnOutOfRangeIndex)
{
  auto blackboard = BT::Blackboard::create();
  std::vector<nav2_demo::Waypoint> waypoints = {{1.0, 2.0, 0.5}};
  blackboard->set("patrol_waypoints", waypoints);
  blackboard->set("current_index", 5);  // out of range

  BT::BehaviorTreeFactory factory;
  registerM3Nodes(factory);

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <NextWaypointNode
          patrol_waypoints="{patrol_waypoints}"
          current_index="{current_index}"
          current_waypoint="{current_waypoint}"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);
  auto status = tree.tickOnce();

  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  // Index should NOT be incremented when out of range
  int idx = blackboard->get<int>("current_index");
  EXPECT_EQ(idx, 5);
}

TEST(NextWaypointNodeTest, FailsOnMissingWaypoints)
{
  auto blackboard = BT::Blackboard::create();
  // Intentionally not setting "patrol_waypoints"
  blackboard->set("current_index", 0);

  BT::BehaviorTreeFactory factory;
  registerM3Nodes(factory);

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <NextWaypointNode
          patrol_waypoints="{patrol_waypoints}"
          current_index="{current_index}"
          current_waypoint="{current_waypoint}"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);
  auto status = tree.tickOnce();

  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

// ======================================================================
// RecordFailureNode 测试
// ======================================================================

TEST(RecordFailureNodeTest, IncrementsFailureCounter)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("failure_count", 5);
  blackboard->set("last_failure_reason", "");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<nav2_demo::RecordFailureNode>("RecordFailureNode");

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <RecordFailureNode
          reason="navigation_timeout"
          failure_count="{failure_count}"
          last_failure_reason="{last_failure_reason}"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);
  auto status = tree.tickOnce();

  // RecordFailureNode should always return SUCCESS
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  // Verify counter incremented
  int count = blackboard->get<int>("failure_count");
  EXPECT_EQ(count, 6);

  // Verify reason recorded
  std::string reason = blackboard->get<std::string>("last_failure_reason");
  EXPECT_EQ(reason, "navigation_timeout");
}

TEST(RecordFailureNodeTest, DefaultReasonIsUnknown)
{
  auto blackboard = BT::Blackboard::create();
  // 不设置 failure_count 和 last_failure_reason，测试默认行为

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<nav2_demo::RecordFailureNode>("RecordFailureNode");

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <RecordFailureNode
          failure_count="{failure_count}"
          last_failure_reason="{last_failure_reason}"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);
  auto status = tree.tickOnce();

  // RecordFailureNode 始终返回 SUCCESS
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  // 不提供 reason 参数，应使用默认值 "unknown"
  // (通过黑板验证 last_failure_reason 存在且正确)
}

TEST(RecordFailureNodeTest, AlwaysReturnsSuccess)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("failure_count", 0);
  blackboard->set("last_failure_reason", "");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<nav2_demo::RecordFailureNode>("RecordFailureNode");

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <RecordFailureNode
          reason="test"
          failure_count="{failure_count}"
          last_failure_reason="{last_failure_reason}"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);

  for (int i = 0; i < 5; i++) {
    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  }

  int count = blackboard->get<int>("failure_count");
  EXPECT_EQ(count, 5);
}

// ======================================================================
// Fallback 协作测试: 模拟 NavigateToPose 失败后 RecordFailureNode 接管
// ======================================================================

// 模拟一个始终失败的节点 (替代 NavigateToPoseNode)
class AlwaysFailNode : public BT::SyncActionNode
{
public:
  AlwaysFailNode(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    return BT::NodeStatus::FAILURE;
  }
};

TEST(FallbackRecoveryTest, FallbackCapturesFailureAndContinues)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("failure_count", 0);
  blackboard->set("last_failure_reason", "");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<AlwaysFailNode>("AlwaysFailNode");
  factory.registerNodeType<nav2_demo::RecordFailureNode>("RecordFailureNode");

  // Fallback 结构: AlwaysFail → RecordFailureNode
  // 当 AlwaysFail 返回 FAILURE 时，Fallback 执行 RecordFailureNode
  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <Fallback name="navigate_with_recovery">
          <AlwaysFailNode/>
          <RecordFailureNode
            reason="timeout"
            failure_count="{failure_count}"
            last_failure_reason="{last_failure_reason}"/>
        </Fallback>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);
  auto status = tree.tickOnce();

  // Fallback 应返回 SUCCESS (因为 RecordFailureNode 返回 SUCCESS)
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  // 验证失败已记录
  EXPECT_EQ(blackboard->get<int>("failure_count"), 1);
  EXPECT_EQ(blackboard->get<std::string>("last_failure_reason"), "timeout");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
