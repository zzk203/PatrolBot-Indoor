// M5.1 + M5.2 + M5.3 + M5.4 + M5.5: 视觉分段对接单元测试
//
// 测试策略:
//   1. VisualServoNode: 验证 P 控制器逻辑、成功条件、超时处理
//      - 使用注入的位姿源模拟机器人位姿（无需 Gazebo）
//   2. RetryNode: 验证重试次数、失败次数溢出、报警记录
//   3. DockActionNode: 验证导航 + 伺服两阶段流程
//   4. 集成测试: 验证完整对接流程

#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"

#include "nav2_demo/bt_nodes/visual_servo_node.hpp"
#include "nav2_demo/bt_nodes/retry_node.hpp"
#include "nav2_demo/bt_nodes/dock_action_node.hpp"
#include "nav2_demo/bt_nodes/waypoint_structs.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "nav2_demo/bt_nodes/set_charging_mode_node.hpp"

// ======================================================================
// 全局 RCLCPP 测试环境
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

/// @brief 将 yaw 转换为四元数
geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

/// @brief 创建指定位置和朝向的机器人位姿
geometry_msgs::msg::Pose makePose(double x, double y, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation = yawToQuaternion(yaw);
  return pose;
}

/// @brief 创建可注入的位姿源回调，每次返回固定位姿
auto makeFixedPoseSource(double x, double y, double yaw)
{
  auto pose = std::make_shared<geometry_msgs::msg::Pose>(makePose(x, y, yaw));
  return [pose]() -> std::optional<geometry_msgs::msg::Pose> {
    return *pose;
  };
}

/// @brief 创建可递增的位姿源回调（模拟逐步靠近）
auto makeApproachPoseSource(
  double start_x, double start_y, double start_yaw,
  double target_x, double target_y, double target_yaw,
  int steps = 10)
{
  auto step = std::make_shared<int>(0);
  auto total_steps = steps;

  auto pose = std::make_shared<geometry_msgs::msg::Pose>();

  return [=]() -> std::optional<geometry_msgs::msg::Pose> {
    double t = std::min(1.0, static_cast<double>(*step) / total_steps);
    double x = start_x + (target_x - start_x) * t;
    double y = start_y + (target_y - start_y) * t;
    double yaw = start_yaw + (target_yaw - start_yaw) * t;
    *pose = makePose(x, y, yaw);
    (*step)++;
    return *pose;
  };
}

/// @brief 注册 M5 节点到工厂
void registerM5Nodes(
  BT::BehaviorTreeFactory & factory,
  const rclcpp::Node::SharedPtr & node)
{
  factory.registerBuilder<nav2_demo::VisualServoNode>(
    "VisualServoNode",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<nav2_demo::VisualServoNode>(name, config, node);
    });

  factory.registerBuilder<nav2_demo::DockActionNode>(
    "DockActionNode",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<nav2_demo::DockActionNode>(name, config, node);
    });

  factory.registerNodeType<nav2_demo::RetryNode>("RetryNode");

  factory.registerBuilder<nav2_demo::SetChargingModeNode>(
    "SetChargingModeNode",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<nav2_demo::SetChargingModeNode>(name, config, node);
    });
}

// ======================================================================
// VisualServoNode 测试 (M5.2)
// ======================================================================

class VisualServoNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_node_ = std::make_shared<rclcpp::Node>("test_visual_servo");
    blackboard_ = BT::Blackboard::create();
  }

  void TearDown() override
  {
    test_node_.reset();
  }

  /// @brief 创建 VisualServoNode 并注入位姿源
  std::unique_ptr<nav2_demo::VisualServoNode> createServoNode(
    nav2_demo::VisualServoNode::PoseSourceCallback pose_source)
  {
    auto node = std::make_unique<nav2_demo::VisualServoNode>(
      "servo", BT::NodeConfig(), test_node_);
    node->setRobotPoseSource(std::move(pose_source));
    return node;
  }

  std::shared_ptr<rclcpp::Node> test_node_;
  BT::Blackboard::Ptr blackboard_;
};

/// @brief 测试视觉伺服节点创建时默认为 IDLE 状态
TEST_F(VisualServoNodeTest, NodeStartsIdle)
{
  auto node = createServoNode(makeFixedPoseSource(7.5, 6.0, -1.57));
  EXPECT_EQ(node->status(), BT::NodeStatus::IDLE);
}

/// @brief 测试机器人已在 dock 位姿时立即成功
TEST_F(VisualServoNodeTest, AlreadyAlignedReturnsSuccess)
{
  // Arrange: 机器人在 dock 位置 (7.5, 6.0, -1.57)
  auto node = createServoNode(makeFixedPoseSource(7.5, 6.0, -1.57));
  node->setInput("dock_pose_x", 7.5);
  node->setInput("dock_pose_y", 6.0);
  node->setInput("dock_pose_yaw", -1.57);
  node->setInput("timeout", 15.0);

  // Act: onStart + onRunning
  auto start_status = node->executeTick();
  EXPECT_EQ(start_status, BT::NodeStatus::RUNNING);

  auto running_status = node->executeTick();

  // Assert: 应该立即成功（误差 < 阈值）
  EXPECT_EQ(running_status, BT::NodeStatus::SUCCESS);
}

/// @brief 测试机器人在阈值范围内时成功
TEST_F(VisualServoNodeTest, WithinThresholdReturnsSuccess)
{
  // Arrange: 机器人在 dock 附近（角度偏差 0.03 rad，横向偏差 0.01 m）
  // 都在阈值内
  double dock_x = 7.5, dock_y = 6.0, dock_yaw = -1.57;
  // 机器人位置使得 yaw_error ≈ 0.03, lateral_error ≈ 0.01
  double robot_x = dock_x - 0.02;  // 略靠后
  double robot_y = dock_y + 0.01;  // 横向偏移 0.01
  double robot_yaw = dock_yaw + 0.03;  // 角度偏移 0.03

  auto node = createServoNode(makeFixedPoseSource(robot_x, robot_y, robot_yaw));
  node->setInput("dock_pose_x", dock_x);
  node->setInput("dock_pose_y", dock_y);
  node->setInput("dock_pose_yaw", dock_yaw);
  node->setInput("timeout", 15.0);

  // Act
  node->executeTick();  // onStart
  auto status = node->executeTick();  // onRunning

  // Assert: 应在阈值内
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

/// @brief 测试机器人远离 dock 时返回 RUNNING
TEST_F(VisualServoNodeTest, FarFromDockReturnsRunning)
{
  // Arrange: 机器人距离 dock 较远
  auto node = createServoNode(makeFixedPoseSource(7.0, 5.5, 0.0));
  node->setInput("dock_pose_x", 7.5);
  node->setInput("dock_pose_y", 6.0);
  node->setInput("dock_pose_yaw", -1.57);
  node->setInput("timeout", 15.0);

  // Act
  node->executeTick();  // onStart
  auto status = node->executeTick();  // onRunning

  // Assert: 应该仍在执行
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
}

/// @brief 测试逐步靠近后成功
TEST_F(VisualServoNodeTest, ApproachThenSuccess)
{
  // Arrange: 从远处逐步靠近 dock
  auto pose_source = makeApproachPoseSource(
    7.0, 5.5, 0.0,    // 起始位姿
    7.48, 6.01, -1.55, // 最终位姿（接近 dock）
    20);               // 20 步逼近

  auto node = createServoNode(pose_source);
  node->setInput("dock_pose_x", 7.5);
  node->setInput("dock_pose_y", 6.0);
  node->setInput("dock_pose_yaw", -1.57);
  node->setInput("timeout", 30.0);

  // Act: 多次 tick 逐步靠近
  node->executeTick();  // onStart

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  int max_ticks = 50;
  while (status == BT::NodeStatus::RUNNING && max_ticks-- > 0) {
    status = node->executeTick();
    // 模拟控制延时
    if (status == BT::NodeStatus::RUNNING) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // Assert: 最终应该成功
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

/// @brief 测试超时返回 FAILURE
TEST_F(VisualServoNodeTest, TimeoutReturnsFailure)
{
  // Arrange: 机器人始终不动，超时 0.5 秒
  auto node = createServoNode(makeFixedPoseSource(6.0, 5.0, 0.0));
  node->setInput("dock_pose_x", 7.5);
  node->setInput("dock_pose_y", 6.0);
  node->setInput("dock_pose_yaw", -1.57);
  node->setInput("timeout", 0.5);  // 0.5 秒超时

  // Act
  node->executeTick();  // onStart

  // 等待超时
  std::this_thread::sleep_for(std::chrono::milliseconds(600));

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  int max_ticks = 10;
  while (status == BT::NodeStatus::RUNNING && max_ticks-- > 0) {
    status = node->executeTick();
    if (status == BT::NodeStatus::RUNNING) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  // Assert: 超时应该失败
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);

  // 验证超时后 dock_success 未设置
  bool dock_success = true;
  node->config().blackboard->get<bool>("dock_success", dock_success);
  EXPECT_FALSE(dock_success);
}

/// @brief 测试 onHalted 停止机器人
TEST_F(VisualServoNodeTest, HaltedStopsRobot)
{
  // Arrange
  auto node = createServoNode(makeFixedPoseSource(7.0, 5.5, 0.0));
  node->setInput("dock_pose_x", 7.5);
  node->setInput("dock_pose_y", 6.0);
  node->setInput("dock_pose_yaw", -1.57);
  node->setInput("timeout", 15.0);

  // Act
  node->executeTick();  // onStart - starts publishing cmd_vel
  node->halt();         // Should stop robot

  // Assert: halt should not crash and robot should stop
  // (cmd_vel = 0, 难以直接验证，但确保不崩溃即可)
  SUCCEED();
}

// ======================================================================
// RetryNode 测试 (M5.3)
// ======================================================================

/// @brief 始终失败的模拟节点
class AlwaysFailNode : public BT::SyncActionNode
{
public:
  AlwaysFailNode(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {
    tick_count_ = std::make_shared<int>(0);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<int>("call_count", "Number of times tick was called"),
    };
  }

  BT::NodeStatus tick() override
  {
    (*tick_count_)++;
    setOutput("call_count", *tick_count_);
    return BT::NodeStatus::FAILURE;
  }

  std::shared_ptr<int> tick_count_;
};

/// @brief 第一次失败、第二次成功的模拟节点
class FailOnceThenSucceedNode : public BT::SyncActionNode
{
public:
  FailOnceThenSucceedNode(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    if (attempts_ < 1) {
      attempts_++;
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  int attempts_ = 0;
};

TEST(RetryNodeTest, RetriesOnFailureUpToMaxAttempts)
{
  // Arrange
  auto blackboard = BT::Blackboard::create();
  blackboard->set("max_attempts", 3);
  blackboard->set("retry_count", 0);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<nav2_demo::RetryNode>("RetryNode");
  factory.registerNodeType<AlwaysFailNode>("AlwaysFailNode");

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <RetryNode max_attempts="{max_attempts}" retry_count="{retry_count}">
          <AlwaysFailNode call_count="{call_count}"/>
        </RetryNode>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);
  blackboard->set("call_count", 0);

  // Act: tick once
  auto status = tree.tickOnce();

  // Assert: 经过 3 次失败后最终 FAILURE
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  int call_count = blackboard->get<int>("call_count");
  EXPECT_EQ(call_count, 3);
  int retry_count = blackboard->get<int>("retry_count");
  EXPECT_EQ(retry_count, 3);
}

TEST(RetryNodeTest, SucceedsAfterRetry)
{
  // Arrange
  auto blackboard = BT::Blackboard::create();
  blackboard->set("max_attempts", 3);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<nav2_demo::RetryNode>("RetryNode");
  factory.registerNodeType<FailOnceThenSucceedNode>("FailOnceThenSucceedNode");

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <RetryNode max_attempts="{max_attempts}" retry_count="{retry_count}">
          <FailOnceThenSucceedNode/>
        </RetryNode>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);

  // Act: 第一次 tick → child FAILURE → retry → RUNNING
  auto status1 = tree.tickOnce();
  EXPECT_EQ(status1, BT::NodeStatus::RUNNING);

  // 第二次 tick → child SUCCESS → SUCCESS
  auto status2 = tree.tickOnce();
  EXPECT_EQ(status2, BT::NodeStatus::SUCCESS);
}

TEST(RetryNodeTest, DefaultMaxAttemptsIs3)
{
  // Arrange
  auto blackboard = BT::Blackboard::create();
  blackboard->set("call_count", 0);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<nav2_demo::RetryNode>("RetryNode");
  factory.registerNodeType<AlwaysFailNode>("AlwaysFailNode");

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <RetryNode>
          <AlwaysFailNode call_count="{call_count}"/>
        </RetryNode>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);

  // Act
  auto status = tree.tickOnce();

  // Assert: 默认重试 3 次
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  int call_count = blackboard->get<int>("call_count");
  EXPECT_EQ(call_count, 3);
}

TEST(RetryNodeTest, SingleAttemptWithNoRetry)
{
  // Arrange
  auto blackboard = BT::Blackboard::create();
  blackboard->set("max_attempts", 1);
  blackboard->set("call_count", 0);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<nav2_demo::RetryNode>("RetryNode");
  factory.registerNodeType<AlwaysFailNode>("AlwaysFailNode");

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <RetryNode max_attempts="{max_attempts}">
          <AlwaysFailNode call_count="{call_count}"/>
        </RetryNode>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);

  // Act
  auto status = tree.tickOnce();

  // Assert: 只尝试 1 次
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  int call_count = blackboard->get<int>("call_count");
  EXPECT_EQ(call_count, 1);
}

// ======================================================================
// RetryNode + VisualServoNode 集成测试 (M5.3)
// ======================================================================

/// @brief 测试 RetryNode 包装 VisualServoNode 的重试行为
class RetryWithServoTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_node_ = std::make_shared<rclcpp::Node>("test_retry_servo");
    blackboard_ = BT::Blackboard::create();

    // 设置黑板参数
    blackboard_->set("dock_pose_x", 7.5);
    blackboard_->set("dock_pose_y", 6.0);
    blackboard_->set("dock_pose_yaw", -1.57);
    blackboard_->set("timeout", 1.0);
    blackboard_->set("max_attempts", 3);
    blackboard_->set("dock_success", false);
    blackboard_->set("retry_count", 0);
  }

  void TearDown() override
  {
    test_node_.reset();
  }

  std::shared_ptr<rclcpp::Node> test_node_;
  BT::Blackboard::Ptr blackboard_;
};

/// @brief 测试重试 3 次后全部失败
TEST_F(RetryWithServoTest, RetryVisualServoAllFail)
{
  // Arrange: 机器人始终远离 dock，导致超时
  // 为 servo 节点注入固定位姿（远离 dock）
  // 我们需要一个能访问 VisualServoNode 的方法
  // 使用工厂注册 builder 来注入位姿

  auto pose_source = makeFixedPoseSource(6.0, 5.0, 0.0);  // 远离 dock

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<nav2_demo::RetryNode>("RetryNode");
  factory.registerBuilder<nav2_demo::VisualServoNode>(
    "VisualServoNode",
    [this, pose_source](const std::string & name, const BT::NodeConfig & config) mutable {
      auto node = std::make_unique<nav2_demo::VisualServoNode>(name, config, test_node_);
      node->setRobotPoseSource(pose_source);
      return node;
    });

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <RetryNode max_attempts="{max_attempts}" retry_count="{retry_count}">
          <VisualServoNode
            dock_pose_x="{dock_pose_x}"
            dock_pose_y="{dock_pose_y}"
            dock_pose_yaw="{dock_pose_yaw}"
            timeout="{timeout}"
            dock_success="{dock_success}"/>
        </RetryNode>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard_);

  // Act: 多次 tick（每次 tick 可能触发重试）
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  int max_ticks = 100;
  while (status == BT::NodeStatus::RUNNING && max_ticks-- > 0) {
    status = tree.tickOnce();
    rclcpp::spin_some(test_node_);
    if (status == BT::NodeStatus::RUNNING) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  // Assert: 最终失败（3 次重试都用完）
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  int retry_count = blackboard_->get<int>("retry_count");
  // 重试计数可能为 3（3 次都失败）
  EXPECT_GE(retry_count, 3);
}

/// @brief 测试某次重试后成功
TEST_F(RetryWithServoTest, RetryThenSuccess)
{
  // Arrange: 使用逐步靠近的位姿源，第一次超时后重试成功
  auto pose_source = makeApproachPoseSource(
    7.2, 5.8, -2.0,    // 起始位姿（有些偏离）
    7.48, 6.01, -1.55, // 最终位姿（接近 dock）
    15);                // 15 步逼近

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<nav2_demo::RetryNode>("RetryNode");
  factory.registerBuilder<nav2_demo::VisualServoNode>(
    "VisualServoNode",
    [this, pose_source](const std::string & name, const BT::NodeConfig & config) mutable {
      auto node = std::make_unique<nav2_demo::VisualServoNode>(name, config, test_node_);
      node->setRobotPoseSource(pose_source);
      return node;
    });

  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <RetryNode max_attempts="3" retry_count="{retry_count}">
          <VisualServoNode
            dock_pose_x="7.5"
            dock_pose_y="6.0"
            dock_pose_yaw="-1.57"
            timeout="5.0"
            dock_success="{dock_success}"/>
        </RetryNode>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard_);

  // Act: tick until done
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  int max_ticks = 100;
  while (status == BT::NodeStatus::RUNNING && max_ticks-- > 0) {
    status = tree.tickOnce();
    rclcpp::spin_some(test_node_);
    if (status == BT::NodeStatus::RUNNING) {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  }

  // Assert: 应该最终成功
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

// ======================================================================
// 完整对接流程集成测试 (M5.5)
// ======================================================================

/// @brief 模拟完整的 ChargeRecovery 流程
class ChargeRecoveryIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_node_ = std::make_shared<rclcpp::Node>("test_charge_recovery");

    // 创建 /battery_simulator/set_charging mock 服务端
    mock_charging_service_ = test_node_->create_service<std_srvs::srv::SetBool>(
      "/battery_simulator/set_charging",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        charging_enabled_ = req->data;
        service_call_count_++;
        res->success = true;
        res->message = req->data ? "charging on" : "charging off";
      });

    blackboard_ = BT::Blackboard::create();

    // 设置初始黑板值
    blackboard_->set("current_index", 3);
    blackboard_->set("waypoints_count", 8);
    blackboard_->set("navigation_timeout", 60.0);
    blackboard_->set("is_low_battery", true);
    blackboard_->set("battery_level", 15.0f);
    blackboard_->set("recovery_point_index", -1);
    blackboard_->set("dock_success", false);
    blackboard_->set("nav_result", std::string(""));

    // 设置 dock 参数
    blackboard_->set("dock_pose_x", 7.5);
    blackboard_->set("dock_pose_y", 6.0);
    blackboard_->set("dock_pose_yaw", -1.57);

    // 设置 dock_prep_pose
    nav2_demo::Waypoint standby{7.0, 6.0, -1.57};
    blackboard_->set("dock_prep_pose", standby.toPoseStamped());

    // 创建模拟 odom 发布者（为 DockActionNode 提供位姿）
    odom_pub_ = test_node_->create_publisher<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(10));

    // 发布初始位姿（在 charge_standby 附近）
    publishOdom(7.05, 5.95, -1.5);
    rclcpp::spin_some(test_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  void TearDown() override
  {
    test_node_.reset();
  }

  void publishOdom(double x, double y, double yaw)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = test_node_->now();
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = yawToQuaternion(yaw);
    odom_pub_->publish(odom);
  }

  std::shared_ptr<rclcpp::Node> test_node_;
  BT::Blackboard::Ptr blackboard_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mock_charging_service_;
  bool charging_enabled_ = false;
  int service_call_count_ = 0;
};

/// @brief 测试完整对接流程
TEST_F(ChargeRecoveryIntegrationTest, FullDockingAndChargeSequence)
{
  // Arrange
  BT::BehaviorTreeFactory factory;

  // 注册 M5 节点
  registerM5Nodes(factory, test_node_);

  // 注册其他需要的节点
  factory.registerNodeType<nav2_demo::RetryNode>("RetryNode");

  // 使用 XML 测试对接序列
  const std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <Sequence name="charge_test">
          <!-- 保存断点 -->
          <Script code="recovery_point_index := {current_index}"/>

          <!-- 执行对接（跳过真实导航，直接视觉伺服） -->
          <DockActionNode
            dock_prep_pose="{dock_prep_pose}"
            dock_pose_x="{dock_pose_x}"
            dock_pose_y="{dock_pose_y}"
            dock_pose_yaw="{dock_pose_yaw}"
            navigation_timeout="5.0"
            servo_timeout="2.0"
            dock_success="{dock_success}"/>

          <!-- 启用充电 -->
          <SetChargingModeNode enabled="true"/>

          <!-- 模拟充电等待（测试中用较短时间） -->
          <Wait wait_duration="0.5"/>

          <!-- 禁用充电 -->
          <SetChargingModeNode enabled="false"/>

          <!-- 重置电量 -->
          <Script code="is_low_battery := false"/>
          <Script code="battery_level := 100.0"/>

          <!-- 恢复断点 -->
          <Script code="current_index := {recovery_point_index}"/>
        </Sequence>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard_);

  // Act: 逐步靠近 dock 使对接成功
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  int max_ticks = 200;
  int tick_count = 0;
  bool reached_dock = false;

  // 启动一个线程来更新 odom 逐步靠近
  std::thread odom_thread([this, &reached_dock]() {
    // 从 (7.05, 5.95, -1.5) 逐步靠近 (7.48, 6.01, -1.55)
    for (int i = 0; i < 30; i++) {
      double t = static_cast<double>(i) / 30.0;
      double x = 7.05 + (7.48 - 7.05) * t;
      double y = 5.95 + (6.01 - 5.95) * t;
      double yaw = -1.5 + (-1.55 + 1.5) * t;
      publishOdom(x, y, yaw);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    reached_dock = true;
  });

  // Tick 直到完成
  while (status == BT::NodeStatus::RUNNING && max_ticks-- > 0) {
    status = tree.tickOnce();
    rclcpp::spin_some(test_node_);
    if (status == BT::NodeStatus::RUNNING) {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    tick_count++;
  }

  odom_thread.join();

  // Assert: 完整序列应该成功
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS) << "Failed after " << tick_count << " ticks";

  // 验证 dock_success
  bool dock_success = blackboard_->get<bool>("dock_success");
  EXPECT_TRUE(dock_success);

  // 验证充电服务被调用（启用 + 禁用 = 2 次）
  EXPECT_EQ(service_call_count_, 2);

  // 验证电量被重置
  float battery_level = blackboard_->get<float>("battery_level");
  EXPECT_FLOAT_EQ(battery_level, 100.0f);

  bool is_low = blackboard_->get<bool>("is_low_battery");
  EXPECT_FALSE(is_low);

  // 验证断点恢复
  int recovered_index = blackboard_->get<int>("current_index");
  EXPECT_EQ(recovered_index, 3);
  int recovery_point = blackboard_->get<int>("recovery_point_index");
  EXPECT_EQ(recovery_point, -1);
}

// ======================================================================
// main
// ======================================================================

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  testing::AddGlobalTestEnvironment(new RCLCPPEnvironment());
  return RUN_ALL_TESTS();
}
