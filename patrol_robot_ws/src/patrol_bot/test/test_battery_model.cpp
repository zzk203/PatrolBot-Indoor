#include <gtest/gtest.h>
#include "patrol_bot/battery_model.hpp"
#include "patrol_bot/patrol_types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include "patrol_bot_interfaces/msg/patrol_status.hpp"
#include <chrono>
#include <thread>

class BatteryModelTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    void SetUp() override {
        node_ = std::make_shared<rclcpp::Node>("test_battery_node");
        battery_msg_count_ = 0;
        last_percentage_ = 0.0;
        last_power_status_ = 0;

        // 订阅电池话题，用于验证发布的电池状态
        battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
            "/patrol/battery", 10,
            [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
                last_percentage_ = msg->percentage;
                last_power_status_ = msg->power_supply_status;
                battery_msg_count_++;
            });

        // 创建状态发布者，用于发送 PatrolStatus 控制充放电模式
        status_pub_ = node_->create_publisher<patrol_bot_interfaces::msg::PatrolStatus>(
            "/patrol/status", rclcpp::QoS(10).reliable());
    }

    void TearDown() override {
        if (battery_model_) {
            battery_model_->stop();
        }
        battery_model_.reset();
        battery_sub_.reset();
        status_pub_.reset();
        node_.reset();
    }

    void create_model(const patrol_bot::BatteryConfig& config) {
        battery_model_ = std::make_unique<patrol_bot::BatteryModel>(node_.get(), config);
    }

    void send_status(uint8_t state) {
        auto msg = patrol_bot_interfaces::msg::PatrolStatus();
        msg.state = state;
        msg.current_route_index = 0;
        msg.current_waypoint_index = 0;
        msg.battery_level = 0.0f;
        status_pub_->publish(msg);
        // 等待 DDS 传递消息后通过 spin 处理 subscription 回调
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        rclcpp::spin_some(node_);
    }

    // 等待并接收电池消息，超时返回 false
    bool wait_for_battery_msg(double timeout_sec = 3.0) {
        int prev_count = battery_msg_count_;
        auto start = node_->now();
        while ((node_->now() - start).seconds() < timeout_sec) {
            rclcpp::spin_some(node_);
            if (battery_msg_count_ > prev_count) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return false;
    }

    // 等待指定数量的新电池消息
    bool wait_for_n_battery_msgs(int n, double timeout_sec = 5.0) {
        int target_count = battery_msg_count_ + n;
        auto start = node_->now();
        while ((node_->now() - start).seconds() < timeout_sec) {
            rclcpp::spin_some(node_);
            if (battery_msg_count_ >= target_count) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return false;
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<patrol_bot::BatteryModel> battery_model_;

    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Publisher<patrol_bot_interfaces::msg::PatrolStatus>::SharedPtr status_pub_;

    int battery_msg_count_ = 0;
    double last_percentage_ = 0.0;
    uint8_t last_power_status_ = 0;
};

// ==================== 测试用例 ====================

// 测试 1: 初始电量 - 构造后 level_ 等于 config.initial_level
TEST_F(BatteryModelTest, InitialLevel) {
    patrol_bot::BatteryConfig config;
    config.initial_level = 75.0;
    config.moving_rate = 0.0;   // 不放电，保持初始值
    config.charge_rate = 0.0;
    config.low_threshold = 20.0;
    config.recovery_threshold = 95.0;

    create_model(config);
    battery_model_->start();

    // 等待第一个 timer tick 触发并发布消息
    ASSERT_TRUE(wait_for_battery_msg()) << "Failed to receive initial battery message";
    EXPECT_DOUBLE_EQ(last_percentage_, 0.75);
}

// 测试 2: 放电消耗 - 非充电模式下每 tick 电量下降 moving_rate
TEST_F(BatteryModelTest, Discharge) {
    patrol_bot::BatteryConfig config;
    config.initial_level = 80.0;
    config.moving_rate = 0.5;
    config.charge_rate = 2.0;
    config.low_threshold = 20.0;
    config.recovery_threshold = 95.0;

    create_model(config);
    battery_model_->start();

    // 等待 2 个 timer ticks
    ASSERT_TRUE(wait_for_n_battery_msgs(2)) << "Failed to receive discharge messages";
    // level_ = 80.0 - 2 * 0.5 = 79.0
    EXPECT_NEAR(last_percentage_, 0.79, 0.001);
}

// 测试 3: 充电恢复 - CHARGING 状态下每 tick 电量上升 charge_rate
TEST_F(BatteryModelTest, Charging) {
    patrol_bot::BatteryConfig config;
    config.initial_level = 50.0;
    config.moving_rate = 0.5;
    config.charge_rate = 2.0;
    config.low_threshold = 20.0;
    config.recovery_threshold = 95.0;

    create_model(config);
    // 先设置充电状态再启动 timer，确保第一个 tick 在充电模式下运行
    send_status(patrol_bot_interfaces::msg::PatrolStatus::CHARGING);
    battery_model_->start();

    // 等待 1 个 timer tick
    ASSERT_TRUE(wait_for_battery_msg()) << "Failed to receive charging message";
    // level_ = 50.0 + 2.0 = 52.0
    EXPECT_NEAR(last_percentage_, 0.52, 0.001);
}

// 测试 4: 上界钳位 - 充电到超过 100% 时钳位在 100.0
TEST_F(BatteryModelTest, UpperBoundClamping) {
    patrol_bot::BatteryConfig config;
    config.initial_level = 99.5;
    config.moving_rate = 0.5;
    config.charge_rate = 2.0;
    config.low_threshold = 20.0;
    config.recovery_threshold = 95.0;

    create_model(config);
    // 先设置充电状态再启动 timer
    send_status(patrol_bot_interfaces::msg::PatrolStatus::CHARGING);
    battery_model_->start();

    // 等待 1 个 timer tick，level_ = min(99.5 + 2.0, 100.0) = 100.0
    ASSERT_TRUE(wait_for_battery_msg()) << "Failed to receive clamping message";
    EXPECT_DOUBLE_EQ(last_percentage_, 1.0);
    // 确认 power_supply_status 为 CHARGING
    EXPECT_EQ(last_power_status_,
              sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
}

// 测试 5: 下界钳位 - 放电到低于 0% 时钳位在 0.0
TEST_F(BatteryModelTest, LowerBoundClamping) {
    patrol_bot::BatteryConfig config;
    config.initial_level = 0.3;
    config.moving_rate = 0.5;
    config.charge_rate = 2.0;
    config.low_threshold = 20.0;
    config.recovery_threshold = 95.0;

    create_model(config);
    // 确保不是充电状态
    send_status(patrol_bot_interfaces::msg::PatrolStatus::PATROLLING);
    battery_model_->start();

    // 等待 1 个 timer tick，level_ = max(0.3 - 0.5, 0.0) = 0.0
    ASSERT_TRUE(wait_for_battery_msg()) << "Failed to receive clamping message";
    EXPECT_DOUBLE_EQ(last_percentage_, 0.0);
    // 确认 power_supply_status 为 DISCHARGING
    EXPECT_EQ(last_power_status_,
              sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING);
}

// 测试 6: 模式切换 - status_callback 收到 CHARGING 后切换为充电，收到非 CHARGING 后切换为放电
TEST_F(BatteryModelTest, ModeSwitch) {
    patrol_bot::BatteryConfig config;
    config.initial_level = 80.0;
    config.moving_rate = 0.5;
    config.charge_rate = 2.0;
    config.low_threshold = 20.0;
    config.recovery_threshold = 95.0;

    create_model(config);
    // 先设置为非充电状态
    send_status(patrol_bot_interfaces::msg::PatrolStatus::PATROLLING);
    battery_model_->start();

    // 等待 2 个放电 ticks：level_ = 80.0 - 2 * 0.5 = 79.0
    ASSERT_TRUE(wait_for_n_battery_msgs(2)) << "Failed to receive discharge messages";
    EXPECT_NEAR(last_percentage_, 0.79, 0.005);
    EXPECT_EQ(last_power_status_,
              sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING);
    double level_after_discharge = last_percentage_;

    // 切换到充电状态
    send_status(patrol_bot_interfaces::msg::PatrolStatus::CHARGING);

    // 等待足够长时间（约 2 个 timer ticks），验证电量上升方向
    // 不依赖精确 tick 计数，避免 send_status 中的 spin 意外触发 timer
    auto start = node_->now();
    int count_before = battery_msg_count_;
    while ((node_->now() - start).seconds() < 2.5) {
        rclcpp::spin_some(node_);
        if (battery_msg_count_ >= count_before + 1) {
            // 至少收到一个充电 tick，验证电量上升
            EXPECT_GT(last_percentage_, level_after_discharge)
                << "Battery should increase after switching to CHARGING";
            EXPECT_EQ(last_power_status_,
                      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    // 确保至少收到了一个充电消息
    EXPECT_GT(battery_msg_count_, count_before)
        << "No charging message received after mode switch";

    // 再切回放电状态，验证方向反转
    double level_after_charge = last_percentage_;
    send_status(patrol_bot_interfaces::msg::PatrolStatus::PATROLLING);

    start = node_->now();
    count_before = battery_msg_count_;
    while ((node_->now() - start).seconds() < 2.5) {
        rclcpp::spin_some(node_);
        if (battery_msg_count_ >= count_before + 1) {
            EXPECT_LT(last_percentage_, level_after_charge)
                << "Battery should decrease after switching to non-CHARGING";
            EXPECT_EQ(last_power_status_,
                      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    EXPECT_GT(battery_msg_count_, count_before)
        << "No discharge message received after mode switch back";
}

// 测试 7: start/stop - start 后定时器运行，stop 后定时器停止
TEST_F(BatteryModelTest, StartStop) {
    patrol_bot::BatteryConfig config;
    config.initial_level = 80.0;
    config.moving_rate = 0.5;
    config.charge_rate = 2.0;
    config.low_threshold = 20.0;
    config.recovery_threshold = 95.0;

    create_model(config);

    // 验证 start 之前没有消息
    rclcpp::spin_some(node_);
    int count_before_start = battery_msg_count_;

    // start 后等待 timer 触发
    battery_model_->start();
    ASSERT_TRUE(wait_for_battery_msg()) << "Failed to receive message after start";
    EXPECT_GT(battery_msg_count_, count_before_start);

    // 记录当前消息数
    int count_after_start = battery_msg_count_;

    // stop 后等待一段时间，验证没有新消息
    battery_model_->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    rclcpp::spin_some(node_);

    EXPECT_EQ(battery_msg_count_, count_after_start)
        << "Battery messages still received after stop";
}

// 主函数
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
