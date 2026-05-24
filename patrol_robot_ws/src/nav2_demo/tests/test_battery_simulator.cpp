// M4.1: BatterySimulatorNode 单元测试
//
// 测试策略:
//   1. 测试初始电量为 100%
//   2. 测试放电过程（电量随时间递减）
//   3. 测试充电过程（电量随时间递增）
//   4. 测试电量边界 ([0, 100] 钳制)
//   5. 测试服务接口启停充放电
//   6. 测试 BatteryState 消息格式

#include <gtest/gtest.h>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "nav2_demo/battery_simulator_node.hpp"

// ======================================================================
// BatterySimulatorNode 功能测试
// ======================================================================

class BatterySimulatorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    // 使用非默认参数加速测试
    auto options = rclcpp::NodeOptions();
    options.parameter_overrides({
      {"initial_percentage", 100.0},
      {"discharge_rate", 10.0},     // 10%/s 加速放电
      {"charge_rate", 20.0},        // 20%/s 加速充电
      {"publish_rate", 10.0},       // 10Hz 加速更新
    });

    node_ = std::make_shared<nav2_demo::BatterySimulatorNode>(options);

    // 创建执行器并添加节点
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    // 创建订阅者收集电池消息
    battery_msg_ = nullptr;
    sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
      "/battery_state", rclcpp::QoS(10).transient_local(),
      [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        battery_msg_ = msg;
      });
  }

  void TearDown() override
  {
    executor_->remove_node(node_);
    node_.reset();
  }

  /// @brief 自旋一段时间，处理定时器和服务回调
  void spinFor(std::chrono::milliseconds duration)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  /// @brief 等待直到收到电池消息或超时
  bool waitForBatteryMessage(std::chrono::seconds timeout = std::chrono::seconds(3))
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
      executor_->spin_some();
      if (battery_msg_) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  /// @brief 调用充电服务
  bool callSetCharging(bool enabled)
  {
    auto client = node_->create_client<std_srvs::srv::SetBool>(
      "/battery_simulator/set_charging");

    if (!client->wait_for_service(std::chrono::seconds(2))) {
      return false;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;

    auto future = client->async_send_request(request);

    // 自旋等待服务响应
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(3)) {
      executor_->spin_some();
      if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
        return future.get()->success;
      }
    }
    return false;
  }

  std::shared_ptr<nav2_demo::BatterySimulatorNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  sensor_msgs::msg::BatteryState::SharedPtr battery_msg_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_;
};

TEST_F(BatterySimulatorTest, InitialPercentageIs100)
{
  // 验证初始电池状态为 100%
  ASSERT_TRUE(waitForBatteryMessage());
  ASSERT_NE(battery_msg_, nullptr);

  // percentage 范围 [0, 1]
  EXPECT_FLOAT_EQ(battery_msg_->percentage, 1.0f);
  EXPECT_GT(battery_msg_->voltage, 12.0f);  // 接近满电电压
}

TEST_F(BatterySimulatorTest, DischargeDecreasesBattery)
{
  // Arrange: 等待初始消息
  ASSERT_TRUE(waitForBatteryMessage());
  float initial_pct = battery_msg_->percentage;
  ASSERT_FLOAT_EQ(initial_pct, 1.0f);

  // Act: 等待 0.5 秒（10%/s 放电速度，预计下降约 5%）
  spinFor(std::chrono::milliseconds(500));

  // Assert: 电量应下降
  ASSERT_NE(battery_msg_, nullptr);
  EXPECT_LT(battery_msg_->percentage, initial_pct);
  // 由于时间精度，下降应在 3%～9% 之间（即剩余 91%～97%）
  EXPECT_GT(battery_msg_->percentage, 0.85f);
  EXPECT_LT(battery_msg_->percentage, 0.97f);
}

TEST_F(BatterySimulatorTest, ChargeIncreasesBattery)
{
  // Arrange: 先放电到约 80%
  ASSERT_TRUE(waitForBatteryMessage());
  spinFor(std::chrono::milliseconds(2000));
  float after_discharge = battery_msg_->percentage;
  EXPECT_LT(after_discharge, 0.9f);  // 确认已放电

  // Act: 启用充电
  ASSERT_TRUE(callSetCharging(true));

  // 等待充电（20%/s 速度）
  spinFor(std::chrono::milliseconds(500));

  // Assert: 电量应增加
  float after_charge = battery_msg_->percentage;
  EXPECT_GT(after_charge, after_discharge);
}

TEST_F(BatterySimulatorTest, BatteryClampsAt100)
{
  // Arrange: 启用充电，让电池充满
  ASSERT_TRUE(callSetCharging(true));

  // Act: 等待 3 秒（20%/s 速度，从~80%到100%约需 1 秒）
  spinFor(std::chrono::seconds(3));

  // Assert: 电量应钳制在 100%
  ASSERT_NE(battery_msg_, nullptr);
  EXPECT_FLOAT_EQ(battery_msg_->percentage, 1.0f);
}

TEST_F(BatterySimulatorTest, BatteryClampsAtZero)
{
  // Arrange: 禁用充电，使用快速放电
  // 为确保从 100% 开始，先充满
  ASSERT_TRUE(callSetCharging(true));
  spinFor(std::chrono::seconds(3));
  ASSERT_FLOAT_EQ(battery_msg_->percentage, 1.0f);

  // Act: 开始放电
  ASSERT_TRUE(callSetCharging(false));

  // 等待 12 秒（10%/s，从 100% 到 0% 需要 10 秒，12 秒确保到底）
  spinFor(std::chrono::seconds(12));

  // Assert: 电量应钳制在 0%
  ASSERT_NE(battery_msg_, nullptr);
  EXPECT_FLOAT_EQ(battery_msg_->percentage, 0.0f);
}

TEST_F(BatterySimulatorTest, ServiceSetCharging)
{
  // 测试服务接口

  // 先确认当前状态是放电中
  ASSERT_TRUE(waitForBatteryMessage());
  spinFor(std::chrono::milliseconds(300));
  float level_before = battery_msg_->percentage;

  // 启用充电
  EXPECT_TRUE(callSetCharging(true));
  spinFor(std::chrono::milliseconds(500));
  float level_after_charge = battery_msg_->percentage;

  // 充电后电量应高于之前
  EXPECT_GT(level_after_charge, level_before);

  // 禁用充电
  EXPECT_TRUE(callSetCharging(false));
  spinFor(std::chrono::milliseconds(300));
  float level_after_discharge = battery_msg_->percentage;

  // 再次启用充电
  EXPECT_TRUE(callSetCharging(true));
  spinFor(std::chrono::milliseconds(500));
  float level_recharged = battery_msg_->percentage;

  // 再次充电后应高于放电后
  EXPECT_GT(level_recharged, level_after_discharge);
}

TEST_F(BatterySimulatorTest, BatteryStateMessageFormat)
{
  // 验证 BatteryState 消息字段完整性
  ASSERT_TRUE(waitForBatteryMessage());
  ASSERT_NE(battery_msg_, nullptr);

  // 基本字段
  EXPECT_FALSE(battery_msg_->header.frame_id.empty());
  EXPECT_GT(battery_msg_->voltage, 0.0f);
  EXPECT_GE(battery_msg_->percentage, 0.0f);
  EXPECT_LE(battery_msg_->percentage, 1.0f);
  EXPECT_TRUE(battery_msg_->present);

  // 状态枚举
  EXPECT_EQ(battery_msg_->power_supply_health,
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD);
  EXPECT_EQ(battery_msg_->power_supply_technology,
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
