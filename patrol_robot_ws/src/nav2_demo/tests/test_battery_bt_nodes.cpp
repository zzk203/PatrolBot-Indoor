// M4.2 + M4.3 + M4.4 + M4.5: 电池相关 BT 节点单元测试
//
// 测试策略:
//   1. BatteryMonitorNode: 验证电量阈值比较、低电标志设置
//   2. IsBatteryLowCondition: 验证黑板标志读取
//   3. SetChargingModeNode: 验证服务调用（需 mock 服务端）
//   4. 集成测试: 完整的 ReactiveSequence + ChargeRecovery 流程验证
//   5. 断点恢复: 验证 current_index 在充电前后的保存与恢复

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"

#include "nav2_demo/bt_nodes/battery_monitor_node.hpp"
#include "nav2_demo/bt_nodes/is_battery_low_condition.hpp"
#include "nav2_demo/bt_nodes/set_charging_mode_node.hpp"
#include "nav2_demo/bt_nodes/next_waypoint_node.hpp"
#include "nav2_demo/bt_nodes/record_failure_node.hpp"
#include "nav2_demo/bt_nodes/waypoint_structs.hpp"
#include "nav2_demo/bt_nodes/load_waypoints_node.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace fs = std::filesystem;

// ======================================================================
// 全局 RCLCPP 测试环境（确保 init/shutdown 只调用一次）
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
// 辅助函数
// ======================================================================

/// @brief 发布电池状态到 /battery_state
void publishBatteryState(
  const rclcpp::Node::SharedPtr & node,
  float percentage)  // 百分比 [0, 100]
{
  auto pub = node->create_publisher<sensor_msgs::msg::BatteryState>(
    "/battery_state", rclcpp::QoS(10).transient_local());

  sensor_msgs::msg::BatteryState msg;
  msg.header.stamp = node->now();
  msg.percentage = percentage / 100.0f;  // [0, 1]
  msg.voltage = 10.0f + percentage / 100.0f * 2.5f;
  msg.present = true;
  msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;

  pub->publish(msg);
  rclcpp::spin_some(node);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  rclcpp::spin_some(node);
}

// ======================================================================
// BatteryMonitorNode 测试（M4.2）
// ======================================================================

class BatteryMonitorNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_node_ = std::make_shared<rclcpp::Node>("test_battery_monitor");
    blackboard_ = BT::Blackboard::create();
  }

  void TearDown() override
  {
    test_node_.reset();
  }

  std::shared_ptr<rclcpp::Node> test_node_;
  BT::Blackboard::Ptr blackboard_;
};

TEST_F(BatteryMonitorNodeTest, ReturnsSuccessWhenBatteryAboveThreshold)
{
  // Arrange
  blackboard_->set("battery_threshold", 20.0f);
  blackboard_->set("battery_level", 0.0f);
  blackboard_->set("is_low_battery", false);

  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<nav2_demo::BatteryMonitorNode>(
    "BatteryMonitorNode",
    [this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<nav2_demo::BatteryMonitorNode>(name, config, test_node_);
    });

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <BatteryMonitorNode
          battery_threshold="{battery_threshold}"
          battery_level="{battery_level}"
          is_low_battery="{is_low_battery}"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard_);

  // Act: 先发布一个高电量消息并等待
  publishBatteryState(test_node_, 80.0f);  // 80%

  auto status = tree.tickOnce();

  // Assert
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  float battery_level = blackboard_->get<float>("battery_level");
  EXPECT_GT(battery_level, 70.0f);  // 应在 80% 左右

  bool is_low = blackboard_->get<bool>("is_low_battery");
  EXPECT_FALSE(is_low);
}

TEST_F(BatteryMonitorNodeTest, ReturnsFailureWhenBatteryBelowThreshold)
{
  // Arrange
  blackboard_->set("battery_threshold", 20.0f);

  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<nav2_demo::BatteryMonitorNode>(
    "BatteryMonitorNode",
    [this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<nav2_demo::BatteryMonitorNode>(name, config, test_node_);
    });

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <BatteryMonitorNode
          battery_threshold="{battery_threshold}"
          battery_level="{battery_level}"
          is_low_battery="{is_low_battery}"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard_);

  // Act: 发布低电量消息
  publishBatteryState(test_node_, 15.0f);  // 15%

  auto status = tree.tickOnce();

  // Assert
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);

  bool is_low = blackboard_->get<bool>("is_low_battery");
  EXPECT_TRUE(is_low);
}

TEST_F(BatteryMonitorNodeTest, ReturnsFailureOnBoundaryThreshold)
{
  // Arrange: 50% threshold
  blackboard_->set("battery_threshold", 50.0f);

  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<nav2_demo::BatteryMonitorNode>(
    "BatteryMonitorNode",
    [this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<nav2_demo::BatteryMonitorNode>(name, config, test_node_);
    });

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <BatteryMonitorNode
          battery_threshold="{battery_threshold}"
          battery_level="{battery_level}"
          is_low_battery="{is_low_battery}"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard_);

  // Act: 发布 49%（低于阈值）
  publishBatteryState(test_node_, 49.0f);
  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);

  // Act: 发布 50%（等于阈值，strictly < 故 FAILURE）
  publishBatteryState(test_node_, 50.0f);
  status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);

  // Act: 发布 51%（高于阈值）
  publishBatteryState(test_node_, 51.0f);
  status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

// ======================================================================
// IsBatteryLowCondition 测试（M4.3）
// ======================================================================

TEST(IsBatteryLowConditionTest, ReturnsSuccessWhenLowBattery)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("is_low_battery", true);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<nav2_demo::IsBatteryLowCondition>("IsBatteryLowCondition");

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <IsBatteryLowCondition is_low_battery="{is_low_battery}"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);
  auto status = tree.tickOnce();

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST(IsBatteryLowConditionTest, ReturnsFailureWhenBatteryOk)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("is_low_battery", false);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<nav2_demo::IsBatteryLowCondition>("IsBatteryLowCondition");

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <IsBatteryLowCondition is_low_battery="{is_low_battery}"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);
  auto status = tree.tickOnce();

  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST(IsBatteryLowConditionTest, DefaultIsFalse)
{
  auto blackboard = BT::Blackboard::create();

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<nav2_demo::IsBatteryLowCondition>("IsBatteryLowCondition");

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <IsBatteryLowCondition is_low_battery="{is_low_battery}"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);
  auto status = tree.tickOnce();

  // 未设置时默认为 false，因此返回 FAILURE
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

// ======================================================================
// 集成测试: ReactiveSequence + 电池检查 + 选择器（M4.3）
// ======================================================================

class BatteryReactiveSequenceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_node_ = std::make_shared<rclcpp::Node>("test_battery_reactive");
    blackboard_ = BT::Blackboard::create();

    // 设置初始黑板值
    blackboard_->set("battery_threshold", 20.0f);
    blackboard_->set("battery_level", 100.0f);
    blackboard_->set("is_low_battery", false);
    blackboard_->set("patrol_complete", false);
    blackboard_->set("recovery_point_index", -1);
  }

  void TearDown() override
  {
    test_node_.reset();
  }

  std::shared_ptr<rclcpp::Node> test_node_;
  BT::Blackboard::Ptr blackboard_;
};

// 模拟电池正常巡逻路径的条件节点
TEST_F(BatteryReactiveSequenceTest, PatrolProceedsWhenBatteryOk)
{
  // Arrange: 发布正常电量
  publishBatteryState(test_node_, 80.0f);

  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<nav2_demo::BatteryMonitorNode>(
    "BatteryMonitorNode",
    [this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<nav2_demo::BatteryMonitorNode>(name, config, test_node_);
    });
  factory.registerNodeType<nav2_demo::IsBatteryLowCondition>("IsBatteryLowCondition");

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <Sequence name="init">
          <Script code="current_index := 5"/>
          <Script code="waypoints_count := 10"/>

          <ReactiveSequence name="main_loop">
            <BatteryMonitorNode
              battery_threshold="{battery_threshold}"
              battery_level="{battery_level}"
              is_low_battery="{is_low_battery}"/>

            <Selector name="charge_or_patrol">
              <Sequence name="charge_branch">
                <IsBatteryLowCondition is_low_battery="{is_low_battery}"/>
                <Script code="charge_executed := true"/>
              </Sequence>
              <Sequence name="patrol_branch">
                <AlwaysSuccess/>
                <Script code="patrol_executed := true"/>
              </Sequence>
            </Selector>
          </ReactiveSequence>
        </Sequence>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard_);

  // Act
  rclcpp::spin_some(test_node_);
  auto status = tree.tickOnce();

  // Assert: 电池正常 → 选择巡逻分支
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(blackboard_->get<bool>("patrol_executed"));
  EXPECT_FALSE(blackboard_->get<bool>("is_low_battery"));
}

// 模拟低电时选择回充路径
TEST_F(BatteryReactiveSequenceTest, ChargeBranchSelectedWhenLowBattery)
{
  // Arrange: 发布低电量
  publishBatteryState(test_node_, 15.0f);

  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<nav2_demo::BatteryMonitorNode>(
    "BatteryMonitorNode",
    [this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<nav2_demo::BatteryMonitorNode>(name, config, test_node_);
    });
  factory.registerNodeType<nav2_demo::IsBatteryLowCondition>("IsBatteryLowCondition");

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <Sequence name="init">
          <Script code="current_index := 3"/>
          <Script code="waypoints_count := 10"/>

          <ReactiveSequence name="main_loop">
            <BatteryMonitorNode
              battery_threshold="{battery_threshold}"
              battery_level="{battery_level}"
              is_low_battery="{is_low_battery}"/>

            <Selector name="charge_or_patrol">
              <Sequence name="charge_branch">
                <IsBatteryLowCondition is_low_battery="{is_low_battery}"/>
                <Script code="charge_executed := true"/>
              </Sequence>
              <Sequence name="patrol_branch">
                <AlwaysSuccess/>
                <Script code="patrol_executed := true"/>
              </Sequence>
            </Selector>
          </ReactiveSequence>
        </Sequence>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard_);

  // Act
  rclcpp::spin_some(test_node_);
  auto status = tree.tickOnce();

  // Assert: 低电 → 选择回充分支
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(blackboard_->get<bool>("charge_executed"));
  EXPECT_FALSE(blackboard_->get<bool>("patrol_executed"));
  EXPECT_TRUE(blackboard_->get<bool>("is_low_battery"));
}

// 模拟 ReactiveSequence 在低电时中断巡逻
TEST_F(BatteryReactiveSequenceTest, ReactiveSequenceHaltsOnLowBattery)
{
  // Arrange: 先发布正常电量
  publishBatteryState(test_node_, 80.0f);

  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<nav2_demo::BatteryMonitorNode>(
    "BatteryMonitorNode",
    [this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<nav2_demo::BatteryMonitorNode>(name, config, test_node_);
    });
  factory.registerNodeType<nav2_demo::IsBatteryLowCondition>("IsBatteryLowCondition");

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <Sequence name="init">
          <Script code="current_index := 0"/>
          <Script code="waypoints_count := 5"/>
          <Script code="recovery_point_index := -1"/>

          <ReactiveSequence name="main_loop">
            <BatteryMonitorNode
              battery_threshold="{battery_threshold}"
              battery_level="{battery_level}"
              is_low_battery="{is_low_battery}"/>

            <Selector name="charge_or_patrol">
              <Sequence name="charge_branch">
                <IsBatteryLowCondition is_low_battery="{is_low_battery}"/>
                <Script code="recovery_point_index := {current_index}"/>
                <Script code="charger_activated := true"/>
              </Sequence>
              <Sequence name="patrol_branch">
                <!-- 模拟巡逻：增加 current_index -->
                <Script code="current_index := {current_index} + 1"/>
                <Script code="patrol_count := {patrol_count} + 1"/>
              </Sequence>
            </Selector>
          </ReactiveSequence>
        </Sequence>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard_);
  blackboard_->set("patrol_count", 0);
  blackboard_->set("charger_activated", false);

  // Act - Tick 1: 电量正常，执行巡逻
  rclcpp::spin_some(test_node_);
  auto status = tree.tickOnce();
  rclcpp::spin_some(test_node_);
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard_->get<int>("current_index"), 1);
  EXPECT_EQ(blackboard_->get<int>("patrol_count"), 1);

  // Tick 2: 电量正常，继续巡逻
  status = tree.tickOnce();
  rclcpp::spin_some(test_node_);
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard_->get<int>("current_index"), 2);
  EXPECT_EQ(blackboard_->get<int>("patrol_count"), 2);

  // Act - Tick 3: 变为低电
  publishBatteryState(test_node_, 15.0f);

  status = tree.tickOnce();
  rclcpp::spin_some(test_node_);

  // Assert: 低电 → 中断巡逻 → 执行回充
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(blackboard_->get<bool>("charger_activated"));
  // index 在进入回充前保存了当前值 2
  EXPECT_EQ(blackboard_->get<int>("recovery_point_index"), 2);
  // 巡逻计数器不应再增加
  EXPECT_EQ(blackboard_->get<int>("patrol_count"), 2);
}

// ======================================================================
// 断点恢复测试（M4.5）
// ======================================================================

TEST_F(BatteryReactiveSequenceTest, ResumePatrolFromRecoveryPoint)
{
  // Arrange: 模拟从回充恢复的场景
  // 发布正常电量（恢复状态）
  publishBatteryState(test_node_, 100.0f);

  blackboard_->set("current_index", 2);       // 从索引 2 恢复
  blackboard_->set("waypoints_count", 5);     // 总共 5 个点
  blackboard_->set("recovery_point_index", -1);  // 已清除
  blackboard_->set("is_low_battery", false);
  blackboard_->set("resume_count", 0);

  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<nav2_demo::BatteryMonitorNode>(
    "BatteryMonitorNode",
    [this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<nav2_demo::BatteryMonitorNode>(name, config, test_node_);
    });
  factory.registerNodeType<nav2_demo::IsBatteryLowCondition>("IsBatteryLowCondition");

  // 模拟 PatrolRound: 计算剩余并执行
  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <Sequence name="test_sequence">
          <!-- PatrolRound 入口：计算剩余 -->
          <Script code="remaining := {waypoints_count} - {current_index}"/>

          <!-- 模拟巡逻循环 -->
          <Repeat num_cycles="{remaining}">
            <Sequence>
              <Script code="resume_count := {resume_count} + 1"/>
              <Script code="current_index := {current_index} + 1"/>
            </Sequence>
          </Repeat>
        </Sequence>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard_);

  // Act
  rclcpp::spin_some(test_node_);
  auto status = tree.tickOnce();

  // Assert
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  // 应从索引 2 开始，剩余 3 个点
  EXPECT_EQ(blackboard_->get<int>("resume_count"), 3);
  EXPECT_EQ(blackboard_->get<int>("current_index"), 5);  // 2 + 3 = 5
}

// ======================================================================
// SetChargingModeNode 测试（需要 mock 服务端）
// ======================================================================

class SetChargingModeNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_node_ = std::make_shared<rclcpp::Node>("test_set_charging");
    blackboard_ = BT::Blackboard::create();

    // 创建 mock 充电服务端（在同一节点上）
    mock_service_ = test_node_->create_service<std_srvs::srv::SetBool>(
      "/battery_simulator/set_charging",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        last_charging_request_ = req->data;
        service_call_count_++;
        res->success = true;
        res->message = req->data ? "charging on" : "charging off";
      });

    rclcpp::spin_some(test_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  void TearDown() override
  {
    test_node_.reset();
  }

  std::shared_ptr<rclcpp::Node> test_node_;
  BT::Blackboard::Ptr blackboard_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mock_service_;
  bool last_charging_request_{false};
  int service_call_count_{0};
};

TEST_F(SetChargingModeNodeTest, EnablesCharging)
{
  // Arrange
  blackboard_->set("enabled", true);

  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<nav2_demo::SetChargingModeNode>(
    "SetChargingModeNode",
    [this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<nav2_demo::SetChargingModeNode>(name, config, test_node_);
    });

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <SetChargingModeNode enabled="{enabled}"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard_);

  // Act
  auto status = tree.tickOnce();

  // Assert
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(last_charging_request_);
  EXPECT_EQ(service_call_count_, 1);
}

TEST_F(SetChargingModeNodeTest, DisablesCharging)
{
  // Arrange
  blackboard_->set("enabled", false);

  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<nav2_demo::SetChargingModeNode>(
    "SetChargingModeNode",
    [this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<nav2_demo::SetChargingModeNode>(name, config, test_node_);
    });

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <SetChargingModeNode enabled="{enabled}"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard_);

  // Act
  auto status = tree.tickOnce();

  // Assert
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(last_charging_request_);
  EXPECT_EQ(service_call_count_, 1);
}

// ======================================================================
// main
// ======================================================================

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // 注册全局 RCLCPP 测试环境
  testing::AddGlobalTestEnvironment(new RCLCPPEnvironment());

  return RUN_ALL_TESTS();
}
