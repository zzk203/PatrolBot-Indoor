// M6: 轮次调度与完整任务编排单元测试
//
// 测试策略:
//   1. IsPatrolCompleteCondition: 验证 completion 标志的读取
//   2. PatrolRound completion: 验证所有点访问完后 patrol_complete 被设置
//   3. 轮次重置逻辑: 验证索引归零、轮次计数递增、标志复位
//   4. 断点续巡逻 (M6.5): 验证充电恢复后从断点继续
//   5. 整轮循环集成测试: 验证 PatrolRound → ReturnToDock → Wait → Reset 流程
//
// 测试原则:
// - 所有节点可通过纯黑板操作测试，无需 ROS 2 运行时环境
// - NavigateToPoseNode 使用 mock 替代（AlwaysFailOrSucceed 节点）
// - Wait 节点使用短时间替代

#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <memory>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"

#include "nav2_demo/bt_nodes/next_waypoint_node.hpp"
#include "nav2_demo/bt_nodes/record_failure_node.hpp"
#include "nav2_demo/bt_nodes/waypoint_structs.hpp"
#include "nav2_demo/bt_nodes/is_patrol_complete_condition.hpp"

// ======================================================================
// 全局 RCLCPP 测试环境（仅用于需要 ROS 时间的测试）
// ======================================================================
class RCLCPPEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

// ======================================================================
// Mock 节点: 模拟 NavigateToPoseNode（总是成功）
// ======================================================================
class MockNavigateSuccessNode : public BT::SyncActionNode
{
public:
  MockNavigateSuccessNode(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Navigation goal pose"),
      BT::InputPort<double>("timeout", 30.0, "Navigation timeout"),
      BT::OutputPort<std::string>("nav_result", "Navigation result"),
    };
  }

  BT::NodeStatus tick() override
  {
    setOutput("nav_result", std::string("SUCCESS"));
    return BT::NodeStatus::SUCCESS;
  }
};

// ======================================================================
// Mock 节点: 模拟低电中断（第 N 次后失败）
// ======================================================================
class BatteryMonitorMockNode : public BT::ConditionNode
{
public:
  BatteryMonitorMockNode(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<float>("battery_threshold", 20.0, "Battery threshold %"),
      BT::InputPort<float>("battery_level", "Current battery level"),
      BT::InputPort<bool>("is_low_battery", "Low battery flag"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto battery_level = getInput<float>("battery_level").value_or(100.0f);
    auto threshold = getInput<float>("battery_threshold").value_or(20.0f);

    if (battery_level >= threshold) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

// ======================================================================
// 辅助函数
// ======================================================================

/// @brief 注册 M6 所需所有节点到工厂
void registerM6Nodes(BT::BehaviorTreeFactory & factory)
{
  // M3 节点
  factory.registerNodeType<nav2_demo::NextWaypointNode>("NextWaypointNode");
  factory.registerNodeType<nav2_demo::RecordFailureNode>("RecordFailureNode");
  // M6 节点
  factory.registerNodeType<nav2_demo::IsPatrolCompleteCondition>(
    "IsPatrolCompleteCondition");
  // Mock 节点
  factory.registerNodeType<MockNavigateSuccessNode>("MockNavigateSuccessNode");
  factory.registerNodeType<BatteryMonitorMockNode>("BatteryMonitorMockNode");
}

/// @brief 创建测试用的 3 个巡逻点
std::vector<nav2_demo::Waypoint> createTestWaypoints()
{
  return {
    {1.0, 2.0, 0.5},
    {3.0, 4.0, 1.0},
    {5.0, 6.0, 1.5},
  };
}

// ======================================================================
// Test 1: M6.1 IsPatrolCompleteCondition 单元测试
// ======================================================================

TEST(IsPatrolCompleteConditionTest, ReturnsSuccessWhenComplete)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("patrol_complete", true);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<nav2_demo::IsPatrolCompleteCondition>(
    "IsPatrolCompleteCondition");

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <IsPatrolCompleteCondition patrol_complete="{patrol_complete}"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);
  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST(IsPatrolCompleteConditionTest, ReturnsFailureWhenNotComplete)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("patrol_complete", false);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<nav2_demo::IsPatrolCompleteCondition>(
    "IsPatrolCompleteCondition");

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <IsPatrolCompleteCondition patrol_complete="{patrol_complete}"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);
  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST(IsPatrolCompleteConditionTest, ReturnsFailureWhenMissing)
{
  auto blackboard = BT::Blackboard::create();
  // Intentionally not setting patrol_complete

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<nav2_demo::IsPatrolCompleteCondition>(
    "IsPatrolCompleteCondition");

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <IsPatrolCompleteCondition patrol_complete="{patrol_complete}"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);
  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

// ======================================================================
// Test 2: M6.1 PatrolRound 完成时设置 patrol_complete
// ======================================================================

TEST(PatrolRoundCompletionTest, SetsPatrolCompleteAfterAllWaypoints)
{
  auto blackboard = BT::Blackboard::create();
  auto waypoints = createTestWaypoints();
  blackboard->set("patrol_waypoints", waypoints);
  blackboard->set("waypoints_count", static_cast<int>(waypoints.size()));
  blackboard->set("current_index", 0);
  blackboard->set("failure_count", 0);
  blackboard->set("navigation_timeout", 30.0);
  blackboard->set("waypoint_wait_duration", 0.01);  // 极短停留

  // 设置 patrol_complete 初始值
  blackboard->set("patrol_complete", false);

  BT::BehaviorTreeFactory factory;
  registerM6Nodes(factory);

  // 直接测试 PatrolRound 内部逻辑（不含电池监测）
  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <Sequence name="patrol_test">
          <Script code="remaining := {waypoints_count} - {current_index}"/>
          <Repeat name="patrol_loop" num_cycles="{remaining}">
            <Sequence name="visit_single_waypoint">
              <NextWaypointNode
                patrol_waypoints="{patrol_waypoints}"
                current_index="{current_index}"
                current_waypoint="{current_waypoint}"/>
              <MockNavigateSuccessNode
                goal="{current_waypoint}"
                timeout="{navigation_timeout}"
                nav_result="{nav_result}"/>
              <Wait wait_duration="{waypoint_wait_duration}"/>
            </Sequence>
          </Repeat>
          <Script code="patrol_complete := true"/>
        </Sequence>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);

  // Tick 多次直到 PatrolRound 完成
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  int max_ticks = 100;
  while (status == BT::NodeStatus::RUNNING && max_ticks-- > 0) {
    status = tree.tickOnce();
    if (status == BT::NodeStatus::RUNNING) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // PatrolRound 应在所有点完成后返回 SUCCESS
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  // 验证 patrol_complete 被设置
  bool patrol_complete = blackboard->get<bool>("patrol_complete");
  EXPECT_TRUE(patrol_complete);

  // 验证索引已递增到总数
  int index = blackboard->get<int>("current_index");
  EXPECT_EQ(index, static_cast<int>(waypoints.size()));

  // 验证所有点都被访问过
  EXPECT_EQ(index, 3);
}

// ======================================================================
// Test 3: M6.6 轮次重置逻辑
// ======================================================================

TEST(RoundResetTest, ResetsIndexAndIncrementsRoundCount)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("current_index", 5);
  blackboard->set("round_count", 2);
  blackboard->set("failure_count", 3);
  blackboard->set("patrol_complete", true);

  BT::BehaviorTreeFactory factory;
  registerM6Nodes(factory);

  // 模拟一轮结束后的重置
  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <Sequence name="round_reset">
          <Script code="current_index := 0"/>
          <Script code="round_count := {round_count} + 1"/>
          <Script code="failure_count := 0"/>
          <Script code="patrol_complete := false"/>
        </Sequence>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);
  auto status = tree.tickOnce();

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  // 验证索引归零
  int current_index = blackboard->get<int>("current_index");
  EXPECT_EQ(current_index, 0);

  // 验证轮次计数递增
  int round_count = blackboard->get<int>("round_count");
  EXPECT_EQ(round_count, 3);

  // 验证失败计数清零
  int failure_count = blackboard->get<int>("failure_count");
  EXPECT_EQ(failure_count, 0);

  // 验证 patrol_complete 被清除
  bool patrol_complete = blackboard->get<bool>("patrol_complete");
  EXPECT_FALSE(patrol_complete);
}

// ======================================================================
// Test 4: M6.5 断点续巡逻
// ======================================================================

TEST(BreakpointResumeTest, ContinuesFromSavedIndexAfterCharge)
{
  auto blackboard = BT::Blackboard::create();
  auto waypoints = createTestWaypoints();
  blackboard->set("patrol_waypoints", waypoints);
  blackboard->set("waypoints_count", static_cast<int>(waypoints.size()));
  blackboard->set("navigation_timeout", 30.0);
  blackboard->set("waypoint_wait_duration", 0.01);

  // 模拟断点恢复：当前索引为 2（已访问 2 个点，剩余 1 个）
  blackboard->set("current_index", 2);
  blackboard->set("failure_count", 0);
  blackboard->set("patrol_complete", false);

  BT::BehaviorTreeFactory factory;
  registerM6Nodes(factory);

  // 模拟 PatrolRound 从索引 2 开始执行
  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <Sequence name="patrol_test">
          <Script code="remaining := {waypoints_count} - {current_index}"/>
          <Repeat name="patrol_loop" num_cycles="{remaining}">
            <Sequence name="visit_single_waypoint">
              <NextWaypointNode
                patrol_waypoints="{patrol_waypoints}"
                current_index="{current_index}"
                current_waypoint="{current_waypoint}"/>
              <MockNavigateSuccessNode
                goal="{current_waypoint}"
                timeout="{navigation_timeout}"
                nav_result="{nav_result}"/>
              <Wait wait_duration="{waypoint_wait_duration}"/>
            </Sequence>
          </Repeat>
          <Script code="patrol_complete := true"/>
        </Sequence>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);

  // 执行巡逻
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  int max_ticks = 100;
  while (status == BT::NodeStatus::RUNNING && max_ticks-- > 0) {
    status = tree.tickOnce();
    if (status == BT::NodeStatus::RUNNING) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  // 验证：巡逻完成后 current_index 应为 3（所有点已访问）
  int final_index = blackboard->get<int>("current_index");
  EXPECT_EQ(final_index, static_cast<int>(waypoints.size()));

  // 验证 patrol_complete 已设置
  bool patrol_complete = blackboard->get<bool>("patrol_complete");
  EXPECT_TRUE(patrol_complete);

  // 验证：一轮内索引从 2 递增到 3，未被重置为 0
  EXPECT_EQ(final_index, 3);
}

// ======================================================================
// Test 5: M6.1 + M6.6 Inverter + IsPatrolCompleteCondition 跳出巡逻循环
// ======================================================================

TEST(PatrolLoopExitTest, InverterBreaksRepeatWhenPatrolComplete)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("patrol_complete", false);
  blackboard->set("dummy_flag", false);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<nav2_demo::IsPatrolCompleteCondition>(
    "IsPatrolCompleteCondition");

  // 模拟: IsPatrolCompleteCondition 在 Inverter 下工作
  // 第一轮: patrol_complete=false → IsPatrolComplete FAILURE → Inverter SUCCESS → 继续
  // Script 模拟巡逻完成设置 patrol_complete=true
  // 第二轮: patrol_complete=true → IsPatrolComplete SUCCESS → Inverter FAILURE → Repeat 终止
  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <Repeat name="inner_loop" num_cycles="1000000">
          <Sequence name="inner_sequence">
            <Inverter name="check_complete">
              <IsPatrolCompleteCondition patrol_complete="{patrol_complete}"/>
            </Inverter>
            <Script code="patrol_complete := true"/>
            <Script code="dummy_flag := true"/>
          </Sequence>
        </Repeat>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);

  // 第一次 tick: Inverter(IsPatrolComplete(FAILURE)) = SUCCESS → Scripts 执行
  // 然后 Repeat 继续（子节点返回 SUCCESS）
  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);

  // patrol_complete 现在为 true

  // 第二次 tick: Inverter(IsPatrolComplete(SUCCESS)) = FAILURE → Sequence FAILURE → Repeat FAILURE
  status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);

  // 验证 patrol_complete 保持 true
  bool patrol_complete = blackboard->get<bool>("patrol_complete");
  EXPECT_TRUE(patrol_complete);

  // 验证 dummy_flag 只在第一次 tick 时设置（第二次被 Inverter 拦截）
  bool dummy = blackboard->get<bool>("dummy_flag");
  EXPECT_TRUE(dummy);
}

// ======================================================================
// Test 6: M6.2 + M6.3 ReturnToDock + Wait 集成（验证流程可通过）
// ======================================================================

TEST(ReturnToDockAndWaitTest, SuccessfullyCompletesReturnAndWait)
{
  auto blackboard = BT::Blackboard::create();

  // 创建 dock_prep_pose
  nav2_demo::Waypoint standby{7.0, 6.0, -1.57};
  blackboard->set("dock_prep_pose", standby.toPoseStamped());
  blackboard->set("navigation_timeout", 30.0);
  blackboard->set("nav_result", std::string(""));

  // M6.3: 轮次间隔
  blackboard->set("round_interval", 0.05);  // 极短间隔用于测试

  BT::BehaviorTreeFactory factory;

  // 注册 NavigateToPose 的 mock
  factory.registerNodeType<MockNavigateSuccessNode>("NavigateToPoseNode");
  // 使用标准 Wait 节点

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <Sequence name="test_sequence">
          <!-- Phase 2: ReturnToDock -->
          <Fallback name="return_to_dock">
            <NavigateToPoseNode
              goal="{dock_prep_pose}"
              timeout="{navigation_timeout}"
              nav_result="{nav_result}"/>
            <AlwaysSuccess/>
          </Fallback>

          <!-- Phase 3: Wait for round interval -->
          <Wait wait_duration="{round_interval}"/>
        </Sequence>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);

  // Tick 直到完成
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  int max_ticks = 50;
  while (status == BT::NodeStatus::RUNNING && max_ticks-- > 0) {
    status = tree.tickOnce();
    if (status == BT::NodeStatus::RUNNING) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  // 验证 nav_result 被设置
  std::string nav_result = blackboard->get<std::string>("nav_result");
  EXPECT_EQ(nav_result, "SUCCESS");
}

// ======================================================================
// Test 7: M6.4 完整一轮循环集成测试
// ======================================================================

TEST(FullRoundCycleTest, CompleteOneRoundPatrolReturnWaitReset)
{
  auto blackboard = BT::Blackboard::create();

  // ===== Setup =====
  auto waypoints = createTestWaypoints();
  blackboard->set("patrol_waypoints", waypoints);
  blackboard->set("waypoints_count", static_cast<int>(waypoints.size()));
  blackboard->set("navigation_timeout", 30.0);
  blackboard->set("waypoint_wait_duration", 0.01);
  blackboard->set("battery_threshold", 20.0f);
  blackboard->set("battery_level", 100.0f);
  blackboard->set("is_low_battery", false);

  nav2_demo::Waypoint standby{7.0, 6.0, -1.57};
  blackboard->set("dock_prep_pose", standby.toPoseStamped());

  blackboard->set("current_index", 0);
  blackboard->set("failure_count", 0);
  blackboard->set("round_count", 0);
  blackboard->set("patrol_complete", false);
  blackboard->set("nav_result", std::string(""));
  blackboard->set("round_interval", 0.05);

  BT::BehaviorTreeFactory factory;
  registerM6Nodes(factory);

  // 模拟完整一轮：PatrolRound(3个点) → ReturnToDock → Wait → Reset
  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <Sequence name="one_round_test">

          <!-- Phase 1: 巡逻（模拟 PatrolRound 内部） -->
          <Sequence name="patrol_phase">
            <Script code="remaining := {waypoints_count} - {current_index}"/>
            <Repeat name="patrol_loop" num_cycles="{remaining}">
              <Sequence name="visit_single_waypoint">
                <NextWaypointNode
                  patrol_waypoints="{patrol_waypoints}"
                  current_index="{current_index}"
                  current_waypoint="{current_waypoint}"/>
                <MockNavigateSuccessNode
                  goal="{current_waypoint}"
                  timeout="{navigation_timeout}"
                  nav_result="{nav_result}"/>
                <Wait wait_duration="{waypoint_wait_duration}"/>
              </Sequence>
            </Repeat>
            <Script code="patrol_complete := true"/>
          </Sequence>

          <!-- Phase 2: ReturnToDock -->
          <Fallback name="return_to_dock">
            <MockNavigateSuccessNode
              goal="{dock_prep_pose}"
              timeout="{navigation_timeout}"
              nav_result="{nav_result}"/>
            <AlwaysSuccess/>
          </Fallback>

          <!-- Phase 3: Wait -->
          <Wait wait_duration="{round_interval}"/>

          <!-- Phase 4: Round Reset -->
          <Script code="current_index := 0"/>
          <Script code="round_count := {round_count} + 1"/>
          <Script code="failure_count := 0"/>
          <Script code="patrol_complete := false"/>

        </Sequence>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);

  // 执行完整一轮
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  int max_ticks = 200;
  int tick_count = 0;
  while (status == BT::NodeStatus::RUNNING && max_ticks-- > 0) {
    status = tree.tickOnce();
    if (status == BT::NodeStatus::RUNNING) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    tick_count++;
  }

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS)
    << "Full round cycle failed after " << tick_count << " ticks";

  // 验证轮次重置:
  // - 索引归零（准备下一轮）
  int current_index = blackboard->get<int>("current_index");
  EXPECT_EQ(current_index, 0) << "Index should be reset to 0";

  // - 轮次计数递增
  int round_count = blackboard->get<int>("round_count");
  EXPECT_EQ(round_count, 1);

  // - patrol_complete 被清除（准备下一轮标记）
  bool patrol_complete = blackboard->get<bool>("patrol_complete");
  EXPECT_FALSE(patrol_complete);

  // - 失败计数清零
  int failure_count = blackboard->get<int>("failure_count");
  EXPECT_EQ(failure_count, 0);
}

// ======================================================================
// Test 8: M6.5 低电中断 → 充电 → 从断点继续 → 完成一轮
// ======================================================================

TEST(BreakpointContinueTest, InterruptChargeAndResumeCompleteRound)
{
  auto blackboard = BT::Blackboard::create();

  // ===== Setup =====
  auto waypoints = createTestWaypoints();
  blackboard->set("patrol_waypoints", waypoints);
  blackboard->set("waypoints_count", static_cast<int>(waypoints.size()));
  blackboard->set("navigation_timeout", 30.0);
  blackboard->set("waypoint_wait_duration", 0.01);

  // 模拟低电场景：电池电量在阈值以下
  blackboard->set("battery_threshold", 20.0f);
  blackboard->set("battery_level", 15.0f);  // 低电
  blackboard->set("is_low_battery", true);

  nav2_demo::Waypoint standby{7.0, 6.0, -1.57};
  blackboard->set("dock_prep_pose", standby.toPoseStamped());

  blackboard->set("current_index", 0);
  blackboard->set("failure_count", 0);
  blackboard->set("round_count", 0);
  blackboard->set("patrol_complete", false);
  blackboard->set("nav_result", std::string(""));
  blackboard->set("round_interval", 0.05);
  blackboard->set("recovery_point_index", -1);

  BT::BehaviorTreeFactory factory;
  registerM6Nodes(factory);

  // 模拟带低电中断的巡逻
  // 前 2 个点电量正常，第 3 个点时低电 → 中断 → 充电 → 恢复 → 完成
  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <Sequence name="one_round_test">

          <!-- 内层巡逻循环（模拟 Repeat/inner_patrol_loop） -->
          <Repeat name="inner_loop" num_cycles="100">
            <Sequence name="inner_sequence">

              <!-- 跳出条件 -->
              <Inverter name="check_complete">
                <IsPatrolCompleteCondition patrol_complete="{patrol_complete}"/>
              </Inverter>

              <!-- 巡逻或回充 -->
              <Fallback name="patrol_or_charge">

                <!-- 巡逻（带低电监测） -->
                <ReactiveSequence name="patrol_with_battery">
                  <BatteryMonitorMockNode
                    battery_threshold="{battery_threshold}"
                    battery_level="{battery_level}"
                    is_low_battery="{is_low_battery}"/>
                  <Sequence name="patrol_round_mock">
                    <Script code="remaining := {waypoints_count} - {current_index}"/>
                    <Repeat name="patrol_loop" num_cycles="{remaining}">
                      <Sequence name="visit_single_waypoint">
                        <NextWaypointNode
                          patrol_waypoints="{patrol_waypoints}"
                          current_index="{current_index}"
                          current_waypoint="{current_waypoint}"/>
                        <MockNavigateSuccessNode
                          goal="{current_waypoint}"
                          timeout="{navigation_timeout}"
                          nav_result="{nav_result}"/>
                        <Wait wait_duration="{waypoint_wait_duration}"/>
                      </Sequence>
                    </Repeat>
                    <Script code="patrol_complete := true"/>
                  </Sequence>
                </ReactiveSequence>

                <!-- 低电回充（模拟 ChargeRecovery） -->
                <Sequence name="mock_charge">
                  <!-- 确认低电 -->
                  <Script code="battery_check := {is_low_battery}"/>
                  <!-- 保存断点（实际上 ReativeSequence 中断后 current_index 已是下一个点） -->
                  <Script code="recovery_point_index := {current_index}"/>
                  <!-- 模拟充电过程 -->
                  <Script code="battery_level := 100.0"/>
                  <Script code="is_low_battery := false"/>
                  <!-- 恢复断点 -->
                  <Script code="current_index := {recovery_point_index}"/>
                  <Script code="recovery_point_index := -1"/>
                  <AlwaysSuccess/>
                </Sequence>

              </Fallback>

            </Sequence>
          </Repeat>

          <!-- ReturnToDock -->
          <Fallback name="return_to_dock">
            <MockNavigateSuccessNode
              goal="{dock_prep_pose}"
              timeout="{navigation_timeout}"
              nav_result="{nav_result}"/>
            <AlwaysSuccess/>
          </Fallback>

          <!-- Wait -->
          <Wait wait_duration="{round_interval}"/>

          <!-- Reset -->
          <Script code="current_index := 0"/>
          <Script code="round_count := {round_count} + 1"/>
          <Script code="failure_count := 0"/>
          <Script code="patrol_complete := false"/>

        </Sequence>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);

  // 模拟：执行巡逻 → 在第 2 个点后低电中断 → 充电 → 继续 → 完成所有点
  // 方案：前 2 次 tick 后手动降低电池，触发低电

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  int max_ticks = 500;
  int tick_count = 0;
  bool low_battery_triggered = false;
  bool charge_completed = false;

  while (status == BT::NodeStatus::RUNNING && max_ticks-- > 0) {
    status = tree.tickOnce();

    // 监视当前索引，在访问第 2 个点后触发低电
    int idx = blackboard->get<int>("current_index");
    if (idx >= 1 && !low_battery_triggered) {
      // 触发低电（当前电池仍是 100%，需要先设低再复位触发回充）
      // 模拟第 2 个点后电量骤降
      if (idx == 1 && !low_battery_triggered) {
        blackboard->set("battery_level", 10.0f);
        blackboard->set("is_low_battery", true);
        low_battery_triggered = true;
      }
    }

    // 检测到充电完成（电池恢复到 100）
    float bl = blackboard->get<float>("battery_level");
    if (low_battery_triggered && bl >= 99.0f && !charge_completed) {
      charge_completed = true;
    }

    if (status == BT::NodeStatus::RUNNING) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    tick_count++;
  }

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS)
    << "Breakpoint-resume test failed after " << tick_count << " ticks";

  // 验证：最终所有点都被访问过（索引从 0 → 3）
  // 重置后索引为 0（下一轮）
  int final_index = blackboard->get<int>("current_index");
  EXPECT_EQ(final_index, 0) << "Index should be 0 after round reset";

  // 验证低电确实被触发过
  EXPECT_TRUE(low_battery_triggered);

  // 验证轮次计数
  int round_count = blackboard->get<int>("round_count");
  EXPECT_EQ(round_count, 1);

  // 验证 patrol_complete 已被清除（准备下一轮）
  bool patrol_complete = blackboard->get<bool>("patrol_complete");
  EXPECT_FALSE(patrol_complete);
}

// ======================================================================
// Test 9: M6.3 Wait 节点读取 blackboard round_interval
// ======================================================================

TEST(RoundIntervalWaitTest, ReadsIntervalFromBlackboard)
{
  auto blackboard = BT::Blackboard::create();

  // 设置轮次间隔
  blackboard->set("round_interval", 0.05);  // 极短间隔

  BT::BehaviorTreeFactory factory;

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <Wait wait_duration="{round_interval}"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);

  BT::NodeStatus status = tree.tickOnce();
  // Wait 第一次 tick 返回 RUNNING（开始计时）
  // 但由于间隔极短，第二次 tick 应返回 SUCCESS
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);

  std::this_thread::sleep_for(std::chrono::milliseconds(60));

  // 第二次 tick: 时间已过，应返回 SUCCESS
  status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

// ======================================================================
// main
// ======================================================================

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  // 添加全局 ROS 环境（仅用于需要 ROS 节点的测试，本例可不依赖）
  // testing::AddGlobalTestEnvironment(new RCLCPPEnvironment());
  return RUN_ALL_TESTS();
}
