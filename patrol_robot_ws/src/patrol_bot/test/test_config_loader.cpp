#include <gtest/gtest.h>
#include "patrol_bot/config_loader.hpp"
#include "patrol_bot/patrol_types.hpp"
#include <fstream>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

class ConfigLoaderTest : public ::testing::Test {
protected:
    std::string tmp_dir;

    void SetUp() override {
        tmp_dir = fs::temp_directory_path() / "patrol_test_config";
        fs::create_directories(tmp_dir);
    }

    void TearDown() override {
        fs::remove_all(tmp_dir);
    }

    std::string write_temp(const std::string& filename, const std::string& content) {
        std::string path = tmp_dir + "/" + filename;
        std::ofstream f(path);
        f << content;
        f.close();
        return path;
    }

    // 辅助方法：期望 load() 抛出包含特定子串的 std::runtime_error
    void expect_error(const std::string& yaml_content, const std::string& expected_substr) {
        std::string path = write_temp("test.yaml", yaml_content);
        try {
            patrol_bot::ConfigLoader::load(path);
            FAIL() << "Expected std::runtime_error containing \"" << expected_substr << "\"";
        } catch (const std::runtime_error& e) {
            std::string msg(e.what());
            EXPECT_NE(msg.find(expected_substr), std::string::npos)
                << "Expected error containing \"" << expected_substr << "\", got: " << msg;
        }
    }
};

// ==================== 测试用例 ====================

// 测试 1: 完整解析正常配置
TEST_F(ConfigLoaderTest, FullValidConfig) {
    std::string yaml = R"(
charging_station:
  x: 1.5
  y: 0.8
  yaw: 0.0

battery:
  initial_level: 100.0
  low_threshold: 20.0
  recovery_threshold: 95.0
  discharge_rate:
    moving: 0.5
  charge_rate: 2.0

camera:
  topic: "/camera/image_raw"
  image_format: "png"
  save_directory: "images"

nav2:
  waypoint_timeout: 120.0

patrol_routes:
  - name: "主通道巡检"
    priority: 1
    waypoints:
      - x: 2.0
        y: 1.0
        yaw: 0.0
        wait_seconds: 5
      - x: 5.0
        y: 3.0
        yaw: 1.57
        wait_seconds: 3
        alarm_simulate:
          type: smoke_detected
          severity: warning

  - name: "办公区巡检"
    priority: 2
    waypoints:
      - x: -2.0
        y: 4.0
        yaw: 3.14
        wait_seconds: 8
        alarm_simulate:
          type: temperature_high
          severity: critical
      - x: 0.0
        y: 6.0
        yaw: -1.57
        wait_seconds: 4
)";

    std::string path = write_temp("full_valid.yaml", yaml);

    patrol_bot::Config config;
    EXPECT_NO_THROW(config = patrol_bot::ConfigLoader::load(path));

    // 验证 charging_station
    EXPECT_DOUBLE_EQ(config.charging_station.x, 1.5);
    EXPECT_DOUBLE_EQ(config.charging_station.y, 0.8);
    EXPECT_DOUBLE_EQ(config.charging_station.yaw, 0.0);

    // 验证 battery
    EXPECT_DOUBLE_EQ(config.battery.initial_level, 100.0);
    EXPECT_DOUBLE_EQ(config.battery.low_threshold, 20.0);
    EXPECT_DOUBLE_EQ(config.battery.recovery_threshold, 95.0);
    EXPECT_DOUBLE_EQ(config.battery.moving_rate, 0.5);
    EXPECT_DOUBLE_EQ(config.battery.charge_rate, 2.0);

    // 验证 camera
    EXPECT_EQ(config.camera.topic, "/camera/image_raw");
    EXPECT_EQ(config.camera.image_format, "png");
    EXPECT_EQ(config.camera.save_directory, "images");

    // 验证 nav2.waypoint_timeout
    EXPECT_DOUBLE_EQ(config.waypoint_timeout, 120.0);

    // 验证 patrol_routes
    ASSERT_EQ(config.routes.size(), 2u);

    // 第一条路线
    const auto& route1 = config.routes[0];
    EXPECT_EQ(route1.name, "主通道巡检");
    EXPECT_EQ(route1.priority, 1);
    ASSERT_EQ(route1.waypoints.size(), 2u);

    EXPECT_DOUBLE_EQ(route1.waypoints[0].pose.x, 2.0);
    EXPECT_DOUBLE_EQ(route1.waypoints[0].pose.y, 1.0);
    EXPECT_DOUBLE_EQ(route1.waypoints[0].pose.yaw, 0.0);
    EXPECT_DOUBLE_EQ(route1.waypoints[0].wait_seconds, 5.0);
    EXPECT_FALSE(route1.waypoints[0].has_alarm);

    EXPECT_DOUBLE_EQ(route1.waypoints[1].pose.x, 5.0);
    EXPECT_DOUBLE_EQ(route1.waypoints[1].pose.y, 3.0);
    EXPECT_DOUBLE_EQ(route1.waypoints[1].pose.yaw, 1.57);
    EXPECT_DOUBLE_EQ(route1.waypoints[1].wait_seconds, 3.0);
    EXPECT_TRUE(route1.waypoints[1].has_alarm);
    EXPECT_EQ(route1.waypoints[1].alarm.type, "smoke_detected");
    EXPECT_EQ(route1.waypoints[1].alarm.severity, "warning");

    // 第二条路线
    const auto& route2 = config.routes[1];
    EXPECT_EQ(route2.name, "办公区巡检");
    EXPECT_EQ(route2.priority, 2);
    ASSERT_EQ(route2.waypoints.size(), 2u);

    EXPECT_DOUBLE_EQ(route2.waypoints[0].pose.x, -2.0);
    EXPECT_DOUBLE_EQ(route2.waypoints[0].pose.y, 4.0);
    EXPECT_DOUBLE_EQ(route2.waypoints[0].pose.yaw, 3.14);
    EXPECT_DOUBLE_EQ(route2.waypoints[0].wait_seconds, 8.0);
    EXPECT_TRUE(route2.waypoints[0].has_alarm);
    EXPECT_EQ(route2.waypoints[0].alarm.type, "temperature_high");
    EXPECT_EQ(route2.waypoints[0].alarm.severity, "critical");

    EXPECT_DOUBLE_EQ(route2.waypoints[1].pose.x, 0.0);
    EXPECT_DOUBLE_EQ(route2.waypoints[1].pose.y, 6.0);
    EXPECT_DOUBLE_EQ(route2.waypoints[1].pose.yaw, -1.57);
    EXPECT_DOUBLE_EQ(route2.waypoints[1].wait_seconds, 4.0);
    EXPECT_FALSE(route2.waypoints[1].has_alarm);
}

// 测试 2: 文件不存在
TEST_F(ConfigLoaderTest, FileNotFound) {
    std::string nonexistent = tmp_dir + "/nonexistent.yaml";
    try {
        patrol_bot::ConfigLoader::load(nonexistent);
        FAIL() << "Expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        std::string msg(e.what());
        EXPECT_NE(msg.find("Config file not found"), std::string::npos);
    }
}

// 测试 3: 缺少 nav2 段
TEST_F(ConfigLoaderTest, MissingNav2) {
    std::string yaml = R"(
charging_station:
  x: 1.0
  y: 2.0
  yaw: 0.0
battery:
  initial_level: 100
  low_threshold: 20
  recovery_threshold: 95
  discharge_rate:
    moving: 0.5
  charge_rate: 2.0
patrol_routes:
  - name: "test"
    priority: 1
    waypoints:
      - x: 0.0
        y: 0.0
        yaw: 0.0
        wait_seconds: 1
)";
    expect_error(yaml, "Missing required field: nav2.waypoint_timeout");
}

// 测试 4: 缺少充电站 x
TEST_F(ConfigLoaderTest, MissingChargingStationX) {
    std::string yaml = R"(
charging_station:
  y: 2.0
  yaw: 0.0
battery:
  initial_level: 100
  low_threshold: 20
  recovery_threshold: 95
  discharge_rate:
    moving: 0.5
  charge_rate: 2.0
nav2:
  waypoint_timeout: 120.0
patrol_routes:
  - name: "test"
    priority: 1
    waypoints:
      - x: 0.0
        y: 0.0
        yaw: 0.0
        wait_seconds: 1
)";
    expect_error(yaml, "Missing required field: charging_station.x");
}

// 测试 5: 缺少电池配置段
TEST_F(ConfigLoaderTest, MissingBattery) {
    std::string yaml = R"(
charging_station:
  x: 1.0
  y: 2.0
  yaw: 0.0
nav2:
  waypoint_timeout: 120.0
patrol_routes:
  - name: "test"
    priority: 1
    waypoints:
      - x: 0.0
        y: 0.0
        yaw: 0.0
        wait_seconds: 1
)";
    expect_error(yaml, "Missing required field: battery");
}

// 测试 6: 缺少 patrol_routes
TEST_F(ConfigLoaderTest, MissingPatrolRoutes) {
    std::string yaml = R"(
charging_station:
  x: 1.0
  y: 2.0
  yaw: 0.0
battery:
  initial_level: 100
  low_threshold: 20
  recovery_threshold: 95
  discharge_rate:
    moving: 0.5
  charge_rate: 2.0
nav2:
  waypoint_timeout: 120.0
)";
    expect_error(yaml, "Missing required field: patrol_routes");
}

// 测试 7: patrol_routes 为空数组
TEST_F(ConfigLoaderTest, EmptyPatrolRoutes) {
    std::string yaml = R"(
charging_station:
  x: 1.0
  y: 2.0
  yaw: 0.0
battery:
  initial_level: 100
  low_threshold: 20
  recovery_threshold: 95
  discharge_rate:
    moving: 0.5
  charge_rate: 2.0
nav2:
  waypoint_timeout: 120.0
patrol_routes: []
)";
    expect_error(yaml, "non-empty");
}

// 测试 8: 摄像头可选字段 - 全部不写，验证使用默认值
TEST_F(ConfigLoaderTest, CameraOptionalDefaults) {
    std::string yaml = R"(
charging_station:
  x: 1.0
  y: 2.0
  yaw: 0.0
battery:
  initial_level: 100
  low_threshold: 20
  recovery_threshold: 95
  discharge_rate:
    moving: 0.5
  charge_rate: 2.0
nav2:
  waypoint_timeout: 120.0
patrol_routes:
  - name: "test"
    priority: 1
    waypoints:
      - x: 0.0
        y: 0.0
        yaw: 0.0
        wait_seconds: 1
)";
    std::string path = write_temp("camera_default.yaml", yaml);

    patrol_bot::Config config;
    EXPECT_NO_THROW(config = patrol_bot::ConfigLoader::load(path));

    // 验证使用 CameraConfig 默认值
    EXPECT_EQ(config.camera.topic, "/camera/image_raw");
    EXPECT_EQ(config.camera.image_format, "png");
    EXPECT_EQ(config.camera.save_directory, "images");
}

// 测试 9: battery low_threshold >= recovery_threshold
TEST_F(ConfigLoaderTest, BatteryThresholdInvalid) {
    std::string yaml = R"(
charging_station:
  x: 1.0
  y: 2.0
  yaw: 0.0
battery:
  initial_level: 100
  low_threshold: 95
  recovery_threshold: 80
  discharge_rate:
    moving: 0.5
  charge_rate: 2.0
nav2:
  waypoint_timeout: 120.0
patrol_routes:
  - name: "test"
    priority: 1
    waypoints:
      - x: 0.0
        y: 0.0
        yaw: 0.0
        wait_seconds: 1
)";
    expect_error(yaml, "battery.low_threshold must be less than battery.recovery_threshold");
}

// 测试 10: battery initial_level 超出范围
TEST_F(ConfigLoaderTest, BatteryInitialLevelOutOfRange) {
    std::string yaml = R"(
charging_station:
  x: 1.0
  y: 2.0
  yaw: 0.0
battery:
  initial_level: 150
  low_threshold: 20
  recovery_threshold: 95
  discharge_rate:
    moving: 0.5
  charge_rate: 2.0
nav2:
  waypoint_timeout: 120.0
patrol_routes:
  - name: "test"
    priority: 1
    waypoints:
      - x: 0.0
        y: 0.0
        yaw: 0.0
        wait_seconds: 1
)";
    expect_error(yaml, "battery.initial_level must be in [0, 100]");
}

// 测试 11: 缺少 waypoint 必填字段 x
TEST_F(ConfigLoaderTest, MissingWaypointX) {
    std::string yaml = R"(
charging_station:
  x: 1.0
  y: 2.0
  yaw: 0.0
battery:
  initial_level: 100
  low_threshold: 20
  recovery_threshold: 95
  discharge_rate:
    moving: 0.5
  charge_rate: 2.0
nav2:
  waypoint_timeout: 120.0
patrol_routes:
  - name: "test"
    priority: 1
    waypoints:
      - y: 0.0
        yaw: 0.0
        wait_seconds: 1
)";
    expect_error(yaml, "Missing required field: waypoint.x");
}

// 测试 12: 缺少 route name
TEST_F(ConfigLoaderTest, MissingRouteName) {
    std::string yaml = R"(
charging_station:
  x: 1.0
  y: 2.0
  yaw: 0.0
battery:
  initial_level: 100
  low_threshold: 20
  recovery_threshold: 95
  discharge_rate:
    moving: 0.5
  charge_rate: 2.0
nav2:
  waypoint_timeout: 120.0
patrol_routes:
  - priority: 1
    waypoints:
      - x: 0.0
        y: 0.0
        yaw: 0.0
        wait_seconds: 1
)";
    expect_error(yaml, "Missing required field: route.name");
}

// 测试 13: 无效 alarm severity
TEST_F(ConfigLoaderTest, InvalidAlarmSeverity) {
    std::string yaml = R"(
charging_station:
  x: 1.0
  y: 2.0
  yaw: 0.0
battery:
  initial_level: 100
  low_threshold: 20
  recovery_threshold: 95
  discharge_rate:
    moving: 0.5
  charge_rate: 2.0
nav2:
  waypoint_timeout: 120.0
patrol_routes:
  - name: "test"
    priority: 1
    waypoints:
      - x: 0.0
        y: 0.0
        yaw: 0.0
        wait_seconds: 1
        alarm_simulate:
          type: smoke_detected
          severity: fatal
)";
    expect_error(yaml, "Invalid alarm severity");
}

// 测试 14: waypoint_timeout <= 0
TEST_F(ConfigLoaderTest, WaypointTimeoutNonPositive) {
    std::string yaml = R"(
charging_station:
  x: 1.0
  y: 2.0
  yaw: 0.0
battery:
  initial_level: 100
  low_threshold: 20
  recovery_threshold: 95
  discharge_rate:
    moving: 0.5
  charge_rate: 2.0
nav2:
  waypoint_timeout: 0
patrol_routes:
  - name: "test"
    priority: 1
    waypoints:
      - x: 0.0
        y: 0.0
        yaw: 0.0
        wait_seconds: 1
)";
    expect_error(yaml, "must be > 0");
}

// 主函数
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
