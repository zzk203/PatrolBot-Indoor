#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "patrol_bot/alarm_manager.hpp"
#include "patrol_bot/patrol_logger.hpp"
#include "patrol_bot_interfaces/msg/patrol_alarm.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>

namespace fs = std::filesystem;

class AlarmManagerTest : public ::testing::Test {
protected:
    std::string tmp_dir;
    rclcpp::Node::SharedPtr node;
    patrol_bot::PatrolLogger* logger;
    patrol_bot::AlarmManager* alarm_mgr;

    // 订阅收到的消息
    patrol_bot_interfaces::msg::PatrolAlarm::SharedPtr last_msg_;
    bool msg_received_;
    rclcpp::Subscription<patrol_bot_interfaces::msg::PatrolAlarm>::SharedPtr sub_;

    void SetUp() override {
        rclcpp::init(0, nullptr);

        tmp_dir = (fs::temp_directory_path() / "patrol_test_alarm").string();
        fs::remove_all(tmp_dir);
        fs::create_directories(tmp_dir);

        node = std::make_shared<rclcpp::Node>("test_alarm_node");
        logger = new patrol_bot::PatrolLogger(tmp_dir);
        alarm_mgr = new patrol_bot::AlarmManager(node.get(), *logger);

        last_msg_.reset();
        msg_received_ = false;

        // 订阅 /patrol/alarm
        sub_ = node->create_subscription<patrol_bot_interfaces::msg::PatrolAlarm>(
            "/patrol/alarm", 10,
            [this](const patrol_bot_interfaces::msg::PatrolAlarm::SharedPtr msg) {
                last_msg_ = msg;
                msg_received_ = true;
            });
    }

    void TearDown() override {
        delete alarm_mgr;
        delete logger;
        node.reset();
        rclcpp::shutdown();
        fs::remove_all(tmp_dir);
    }

    /// @brief 触发一次 spin，等待订阅回调
    void spin_until_received(int timeout_ms = 2000) {
        auto start = std::chrono::steady_clock::now();
        while (rclcpp::ok() && !msg_received_) {
            rclcpp::spin_some(node);
            auto elapsed = std::chrono::steady_clock::now() - start;
            if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > timeout_ms) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    /// @brief 读取日志文件内容
    std::string read_log_file() {
        for (const auto& entry : fs::directory_iterator(tmp_dir)) {
            auto fname = entry.path().filename().string();
            if (fname.find("patrol_") == 0 && fname.size() > 6) {
                std::ifstream f(entry.path());
                if (!f.is_open()) return "";
                std::stringstream ss;
                ss << f.rdbuf();
                return ss.str();
            }
        }
        return "";
    }
};

// ============================================================
// 1. Warning alarm — 验证 topic 消息和日志内容
// ============================================================
TEST_F(AlarmManagerTest, WarningAlarm) {
    alarm_mgr->raise_alarm("smoke_detected", "warning", 1.5, 2.5);
    spin_until_received();

    ASSERT_TRUE(msg_received_) << "未收到报警消息";
    EXPECT_EQ(last_msg_->severity, patrol_bot_interfaces::msg::PatrolAlarm::WARNING);
    EXPECT_EQ(last_msg_->alarm_type, "smoke_detected");
    EXPECT_FLOAT_EQ(last_msg_->x, 1.5f);
    EXPECT_FLOAT_EQ(last_msg_->y, 2.5f);

    // 验证日志
    std::string log_content = read_log_file();
    EXPECT_FALSE(log_content.empty());
    EXPECT_NE(log_content.find("WARNING"), std::string::npos);
    EXPECT_NE(log_content.find("smoke_detected"), std::string::npos);
    EXPECT_NE(log_content.find("1.5"), std::string::npos);
    EXPECT_NE(log_content.find("2.5"), std::string::npos);
}

// ============================================================
// 2. Critical alarm — 验证 topic 消息和日志内容
// ============================================================
TEST_F(AlarmManagerTest, CriticalAlarm) {
    alarm_mgr->raise_alarm("temperature_high", "critical", 10.0, 20.0);
    spin_until_received();

    ASSERT_TRUE(msg_received_) << "未收到报警消息";
    EXPECT_EQ(last_msg_->severity, patrol_bot_interfaces::msg::PatrolAlarm::CRITICAL);
    EXPECT_EQ(last_msg_->alarm_type, "temperature_high");
    EXPECT_FLOAT_EQ(last_msg_->x, 10.0f);
    EXPECT_FLOAT_EQ(last_msg_->y, 20.0f);

    // 验证日志
    std::string log_content = read_log_file();
    EXPECT_FALSE(log_content.empty());
    EXPECT_NE(log_content.find("CRITICAL"), std::string::npos);
    EXPECT_NE(log_content.find("temperature_high"), std::string::npos);
}

// ============================================================
// 3. 消息完整性 — 验证所有字段
// ============================================================
TEST_F(AlarmManagerTest, MessageIntegrity) {
    alarm_mgr->raise_alarm("gas_leak", "warning", 3.14159, 2.71828);
    spin_until_received();

    ASSERT_TRUE(msg_received_) << "未收到报警消息";

    // severity
    EXPECT_EQ(last_msg_->severity, patrol_bot_interfaces::msg::PatrolAlarm::WARNING);

    // alarm_type
    EXPECT_EQ(last_msg_->alarm_type, "gas_leak");

    // 坐标
    EXPECT_FLOAT_EQ(last_msg_->x, 3.14159f);
    EXPECT_FLOAT_EQ(last_msg_->y, 2.71828f);

    // timestamp — 应该是一个非零的时间（epoch > 0 表示已设置）
    EXPECT_GT(last_msg_->timestamp.sec, 0)
        << "timestamp.sec 应大于 0（时间戳未被正确设置）";
}

// ============================================================
// 4. 日志写入 — 验证日志文件内容
// ============================================================
TEST_F(AlarmManagerTest, LogWrite) {
    alarm_mgr->raise_alarm("motion_detected", "warning", 5.0, 8.0);
    spin_until_received();

    // 读取日志文件
    std::string log_content = read_log_file();
    ASSERT_FALSE(log_content.empty()) << "日志文件应为空";

    // 验证日志内容包含报警信息
    EXPECT_NE(log_content.find("motion_detected"), std::string::npos);
    EXPECT_NE(log_content.find("5.0"), std::string::npos);
    EXPECT_NE(log_content.find("8.0"), std::string::npos);

    // 验证日志格式: [timestamp] [WARN] [AlarmManager] ...
    EXPECT_NE(log_content.find("[WARN]"), std::string::npos);
    EXPECT_NE(log_content.find("[AlarmManager]"), std::string::npos);
}
