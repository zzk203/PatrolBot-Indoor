# 室内巡逻机器人系统详细设计文档

## 目录

- [1. 概述](#1-概述)
  - [1.1 文档目的](#11-文档目的)
  - [1.2 设计范围](#12-设计范围)
  - [1.3 与概要设计的关系](#13-与概要设计的关系)
- [2. 模块详细设计](#2-模块详细设计)
  - [2.1 内部数据结构 (patrol_types.hpp)](#21-内部数据结构-patrol_typeshpp)
  - [2.2 配置加载器 (ConfigLoader)](#22-配置加载器-configloader)
  - [2.3 电池模型 (BatteryModel)](#23-电池模型-batterymodel)
  - [2.4 报警管理器 (AlarmManager)](#24-报警管理器-alarmmanager)
  - [2.5 巡逻日志 (PatrolLogger)](#25-巡逻日志-patrollogger)
  - [2.6 导航客户端 (Nav2ActionClient)](#26-导航客户端-nav2actionclient)
  - [2.7 摄像头缓存 (CameraBuffer)](#27-摄像头缓存-camerabuffer)
- [3. 行为树节点详细设计](#3-行为树节点详细设计)
  - [3.1 节点注册与工厂](#31-节点注册与工厂)
  - [3.2 Condition 节点实现](#32-condition-节点实现)
  - [3.3 Action 节点实现](#33-action-节点实现)
  - [3.4 行为树 XML 定义](#34-行为树-xml-定义)
- [4. 系统节点详细设计](#4-系统节点详细设计)
  - [4.1 patrol_bot_node 生命周期](#41-patrol_bot_node-生命周期)
  - [4.2 初始化流程](#42-初始化流程)
  - [4.3 主循环设计](#43-主循环设计)
  - [4.4 Service 回调实现](#44-service-回调实现)
  - [4.5 Topic 发布实现](#45-topic-发布实现)
- [5. 消息定义](#5-消息定义)
  - [5.1 自定义消息文件](#51-自定义消息文件)
  - [5.2 标准消息使用](#52-标准消息使用)
- [6. 配置文件详细规格](#6-配置文件详细规格)
- [7. 构建系统设计](#7-构建系统设计)
  - [7.1 CMakeLists.txt](#71-cmakeliststxt)
  - [7.2 package.xml](#72-packagexml)
- [8. 线程与并发设计](#8-线程与并发设计)
- [9. 错误处理与降级策略](#9-错误处理与降级策略)
- [10. 测试策略](#10-测试策略)
  - [10.1 单元测试](#101-单元测试)
  - [10.2 集成测试](#102-集成测试)
- [11. 附录：设计决策记录](#11-附录设计决策记录)

---

## 1. 概述

### 1.1 文档目的

本文档为室内巡逻机器人系统的详细设计文档，逐模块、逐节点描述接口契约、内部实现逻辑、状态机和边界条件。内容与当前代码实现保持一致，开发人员可据此理解系统全貌，也可作为后续重构的参照基准。

### 1.2 设计范围

涵盖 `patrol_bot` 包内所有 C++ 源码模块、行为树节点、ROS2 接口、消息定义、构建系统和测试策略。不涉及 Nav2 参数调优、Gazebo 世界建模、Docker 编排配置、demo 包（`action_demo`、`topic_demo`、`service_demo`、`bt_demo`、`nav2_demo`）。

### 1.3 与概要设计的关系

本文档继承概要设计文档（`doc/high_level_design.md`）的架构决策和接口定义，在此基础上细化到函数签名、伪代码和分支逻辑。设计决策记录见附录。

---

## 2. 模块详细设计

### 2.1 内部数据结构 (patrol_types.hpp)

**文件**: `include/patrol_bot/patrol_types.hpp`

```cpp
#pragma once

#include <string>
#include <vector>
#include <optional>

namespace patrol_bot {

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct AlarmConfig {
    std::string type;
    std::string severity;
};

struct Waypoint {
    Pose2D pose;
    double wait_seconds = 0.0;
    bool has_alarm = false;
    AlarmConfig alarm;
};

struct Route {
    std::string name;
    int priority = 0;
    std::vector<Waypoint> waypoints;
};

struct BatteryConfig {
    double initial_level = 100.0;
    double low_threshold = 20.0;
    double recovery_threshold = 95.0;
    double moving_rate = 0.5;
    double charge_rate = 2.0;
};

struct CameraConfig {
    std::string topic = "/camera/image_raw";
    std::string image_format = "png";
    std::string save_directory = "images";
};

struct Config {
    Pose2D charging_station;
    BatteryConfig battery;
    std::vector<Route> routes;
    double waypoint_timeout = 120.0;
    CameraConfig camera;
};

enum class PatrolState {
    IDLE = 0,
    PATROLLING = 1,
    PAUSED = 2,
    CHARGING = 3,
    STOPPED = 4
};

}  // namespace patrol_bot
```

> **设计说明**: `PatrolState` 枚举值与 `patrol_bot_interfaces/msg/PatrolStatus.msg` 中的常量定义保持一一对应。

### 2.2 配置加载器 (ConfigLoader)

**文件**: `include/patrol_bot/config_loader.hpp`, `src/config_loader.cpp`

```cpp
#pragma once

#include "patrol_bot/patrol_types.hpp"
#include <string>

namespace patrol_bot {

class ConfigLoader {
public:
    // 从 YAML 文件加载配置，解析失败抛出 std::runtime_error
    static Config load(const std::string& yaml_path);

private:
    static Route parse_route(const YAML::Node& node);
    static Waypoint parse_waypoint(const YAML::Node& node);
    static std::optional<AlarmConfig> parse_alarm(const YAML::Node& node);
};

}  // namespace patrol_bot
```

**实现伪代码**:

```
Config ConfigLoader::load(yaml_path):
    // 1. 文件存在性检查
    if !std::filesystem::exists(yaml_path):
        throw std::runtime_error("Config file not found: " + path)

    // 2. 加载 YAML
    try:
        yaml = YAML::LoadFile(yaml_path)
    catch YAML::Exception:
        throw std::runtime_error("Failed to parse config file")

    Config config

    // 3. nav2.waypoint_timeout (必填)
    if !yaml["nav2"] || !yaml["nav2"]["waypoint_timeout"]:
        throw std::runtime_error("Missing required field: nav2.waypoint_timeout")
    config.waypoint_timeout = yaml["nav2"]["waypoint_timeout"].as<double>()
    if config.waypoint_timeout <= 0:
        throw std::runtime_error("nav2.waypoint_timeout must be > 0")

    // 4. charging_station (必填，逐字段校验)
    if !yaml["charging_station"]:
        throw "Missing required field: charging_station"
    try: config.charging_station.x = yaml["charging_station"]["x"].as<double>()
    catch: throw "Missing required field: charging_station.x"
    try: config.charging_station.y = yaml["charging_station"]["y"].as<double>()
    catch: throw "Missing required field: charging_station.y"
    try: config.charging_station.yaw = yaml["charging_station"]["yaw"].as<double>()
    catch: throw "Missing required field: charging_station.yaw"

    // 5. battery (必填，逐字段校验)
    if !yaml["battery"]:
        throw "Missing required field: battery"
    try: config.battery.initial_level = yaml["battery"]["initial_level"].as<double>()
    catch: throw "Missing required field: battery.initial_level"
    try: config.battery.low_threshold = yaml["battery"]["low_threshold"].as<double>()
    catch: throw "Missing required field: battery.low_threshold"
    try: config.battery.recovery_threshold = yaml["battery"]["recovery_threshold"].as<double>()
    catch: throw "Missing required field: battery.recovery_threshold"
    try: config.battery.moving_rate = yaml["battery"]["discharge_rate"]["moving"].as<double>()
    catch: throw "Missing required field: battery.discharge_rate.moving"
    try: config.battery.charge_rate = yaml["battery"]["charge_rate"].as<double>()
    catch: throw "Missing required field: battery.charge_rate"

    // 值域校验
    if config.battery.low_threshold >= config.battery.recovery_threshold:
        throw "battery.low_threshold must be less than battery.recovery_threshold"
    if config.battery.initial_level < 0 || config.battery.initial_level > 100:
        throw "battery.initial_level must be in [0, 100]"

    // 6. camera (可选，有默认值)
    if yaml["camera"]:
        if yaml["camera"]["topic"]:
            config.camera.topic = yaml["camera"]["topic"].as<string>()
        if yaml["camera"]["image_format"]:
            config.camera.image_format = yaml["camera"]["image_format"].as<string>()
        if yaml["camera"]["save_directory"]:
            config.camera.save_directory = yaml["camera"]["save_directory"].as<string>()

    // 7. patrol_routes (必填，非空数组)
    if !yaml["patrol_routes"]:
        throw "Missing required field: patrol_routes"
    if !yaml["patrol_routes"].IsSequence() || yaml["patrol_routes"].size() == 0:
        throw "patrol_routes must be a non-empty array"
    for route_node in yaml["patrol_routes"]:
        config.routes.push_back(parse_route(route_node))

    return config


Route ConfigLoader::parse_route(node):
    Route route
    try: route.name = node["name"].as<string>()
    catch: throw "Missing required field: route.name"
    try: route.priority = node["priority"].as<int>()
    catch: throw "Missing required field: route.priority"

    if !node["waypoints"] || !node["waypoints"].IsSequence() || size == 0:
        throw "Route \"" + route.name + "\" must have non-empty waypoints"
    for wp_node in node["waypoints"]:
        route.waypoints.push_back(parse_waypoint(wp_node))
    return route


Waypoint ConfigLoader::parse_waypoint(node):
    Waypoint wp
    try: wp.pose.x = node["x"].as<double>()
    catch: throw "Missing required field: waypoint.x"
    try: wp.pose.y = node["y"].as<double>()
    catch: throw "Missing required field: waypoint.y"
    try: wp.pose.yaw = node["yaw"].as<double>()
    catch: throw "Missing required field: waypoint.yaw"
    try: wp.wait_seconds = node["wait_seconds"].as<double>()
    catch: throw "Missing required field: waypoint.wait_seconds"

    // 可选的 alarm_simulate
    alarm_opt = parse_alarm(node["alarm_simulate"])
    if alarm_opt:
        wp.has_alarm = true
        wp.alarm = *alarm_opt
    return wp


std::optional<AlarmConfig> ConfigLoader::parse_alarm(node):
    if !node || !node.IsMap():
        return std::nullopt
    AlarmConfig alarm
    try: alarm.type = node["type"].as<string>()
    catch: return std::nullopt
    try: alarm.severity = node["severity"].as<string>()
    catch: return std::nullopt
    // severity 必须是 "warning" 或 "critical"
    if alarm.severity != "warning" && alarm.severity != "critical":
        throw "Invalid alarm severity: " + alarm.severity
    return alarm
```

**依赖**: `yaml-cpp` (>= 0.7.0)。

**错误处理**: 所有必填字段逐字段 try/catch，每个缺失字段抛出带具体名称的 `std::runtime_error`。字段值域校验（如 initial_level ∈ [0,100]）同样在加载阶段执行。

### 2.3 电池模型 (BatteryModel)

**文件**: `include/patrol_bot/battery_model.hpp`, `src/battery_model.cpp`

```cpp
#pragma once

#include "patrol_bot/patrol_types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include "patrol_bot_interfaces/msg/patrol_status.hpp"

namespace patrol_bot {

class BatteryModel {
public:
    BatteryModel(rclcpp::Node* node,
                 const BatteryConfig& config);

    // 启动内部定时器 (1Hz)，开始周期更新
    void start();

    // 停止定时器
    void stop();

private:
    // 1Hz 定时器回调: 更新电量 + 发布 /patrol/battery
    void timer_callback();

    // /patrol/status 订阅回调: 根据 state 字段切换充放电模式
    void status_callback(const patrol_bot_interfaces::msg::PatrolStatus::SharedPtr msg);

    rclcpp::Node* node_;
    BatteryConfig config_;
    double level_;

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::Subscription<patrol_bot_interfaces::msg::PatrolStatus>::SharedPtr status_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool is_charging_ = false;   // 从 /patrol/status 的 state == CHARGING 推演
};

}  // namespace patrol_bot
```

**实现伪代码**:

```
BatteryModel(node, config):
    level_ = config.initial_level
    config_ = config

    // 发布 /patrol/battery (QoS: reliable, depth=10)
    battery_pub_ = node->create_publisher<BatteryState>(
        "/patrol/battery", rclcpp::QoS(10).reliable())

    // 订阅 /patrol/status (QoS: reliable, depth=10)
    status_sub_ = node->create_subscription<PatrolStatus>(
        "/patrol/status", 10,
        [this](msg) { status_callback(msg); })

start():
    timer_ = node->create_wall_timer(1s, [this]() { timer_callback(); })

stop():
    if timer_:
        timer_->cancel()
        timer_.reset()

timer_callback():
    // 根据充放电模式更新电量
    if is_charging_:
        level_ = min(level_ + config_.charge_rate, 100.0)
    else:
        level_ = max(level_ - config_.moving_rate, 0.0)

    // 构造并发布 BatteryState 消息
    msg = BatteryState()
    msg.header.stamp = node_->now()
    msg.percentage = level_ / 100.0
    if is_charging_:
        msg.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_CHARGING
    else:
        msg.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_DISCHARGING
    msg.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_GOOD
    battery_pub_->publish(msg)

status_callback(msg):
    is_charging_ = (msg->state == PatrolStatus::CHARGING)
```

**生命周期**:
- `start()`: 创建 1Hz 定时器开始运行
- `stop()`: 取消定时器，停止更新。在 `PatrolBotNode::~PatrolBotNode()` 中调用

**设计说明**:
- BatteryModel 通过订阅 `/patrol/status` 感知当前充放电模式（`state == CHARGING` → 充电），自闭环无需外部注入
- 发布 `sensor_msgs/BatteryState` 到 `/patrol/battery`，后续可无缝替换为 Gazebo 电池插件

### 2.4 报警管理器 (AlarmManager)

**文件**: `include/patrol_bot/alarm_manager.hpp`, `src/alarm_manager.cpp`

```cpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include "patrol_bot_interfaces/msg/patrol_alarm.hpp"
#include "patrol_bot/patrol_logger.hpp"

namespace patrol_bot {

class AlarmManager {
public:
    AlarmManager(rclcpp::Node* node, PatrolLogger& logger);

    // 触发报警
    // @param type  报警类型 (smoke_detected, temperature_high 等)
    // @param severity "warning" | "critical"
    // @param x, y  触发位置坐标
    void raise_alarm(const std::string& type,
                     const std::string& severity,
                     double x, double y);

private:
    rclcpp::Publisher<patrol_bot_interfaces::msg::PatrolAlarm>::SharedPtr alarm_pub_;
    PatrolLogger& logger_;
};

}  // namespace patrol_bot
```

**实现伪代码**:

```
AlarmManager(node, logger):
    qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable()
    alarm_pub_ = node->create_publisher<PatrolAlarm>("/patrol/alarm", qos)
    logger_ = logger

raise_alarm(type, severity, x, y):
    msg = PatrolAlarm()

    if severity == "critical":
        msg.severity = PatrolAlarm::CRITICAL
        logger_.error("AlarmManager",
            "CRITICAL alarm: type=" + type +
            " at (" + to_string(x) + ", " + to_string(y) + ")")
    else:
        msg.severity = PatrolAlarm::WARNING
        logger_.warn("AlarmManager",
            "WARNING alarm: type=" + type +
            " at (" + to_string(x) + ", " + to_string(y) + ")")

    msg.alarm_type = type
    msg.x = static_cast<float>(x)
    msg.y = static_cast<float>(y)
    msg.timestamp = rclcpp::Clock().now()

    alarm_pub_->publish(msg)
```

> **注意**: AlarmManager 不负责修改 `patrol_state`。`critical` 异常导致暂停的逻辑由 BT 节点 `HandleAlarm` 执行，以维持模块间职责清晰。

### 2.5 巡逻日志 (PatrolLogger)

**文件**: `include/patrol_bot/patrol_logger.hpp`, `src/patrol_logger.cpp`

```cpp
#pragma once

#include <string>
#include <fstream>
#include <mutex>

namespace patrol_bot {

class PatrolLogger {
public:
    // @param log_dir 日志输出目录，文件自动命名为 patrol_YYYY-MM-DD_HH-MM-SS.log
    explicit PatrolLogger(const std::string& log_dir);

    ~PatrolLogger();

    void info(const std::string& tag, const std::string& message);
    void warn(const std::string& tag, const std::string& message);
    void error(const std::string& tag, const std::string& message);

private:
    std::string format(const std::string& level,
                       const std::string& tag,
                       const std::string& message);
    void write(const std::string& level,
               const std::string& tag,
               const std::string& message);
    std::string timestamp() const;

    std::ofstream file_;
    std::mutex mutex_;
};

}  // namespace patrol_bot
```

**实现伪代码**:

```
PatrolLogger(log_dir):
    std::filesystem::create_directories(log_dir)

    now = system_clock::now()
    time_t = system_clock::to_time_t(now)
    tm = localtime(&time_t)

    // 文件名: logs/patrol_2026-05-25_14-30-01.log
    filename = format("patrol_%Y-%m-%d_%H-%M-%S.log", tm)

    file_.open(log_dir + "/" + filename)
    if !file_.is_open():
        throw std::runtime_error("Failed to create log file")

~PatrolLogger():
    if file_.is_open():
        file_.close()

info(tag, msg):  write("INFO",  tag, msg)
warn(tag, msg):  write("WARN",  tag, msg)
error(tag, msg): write("ERROR", tag, msg)

write(level, tag, msg):
    line = format(level, tag, msg)
    {
        lock_guard<mutex> lock(mutex_)
        std::cout << line << std::endl    // 终端输出
        file_ << line << std::endl        // 文件输出
        file_.flush()                     // 立即落盘
    }

format(level, tag, msg):
    return "[" + timestamp() + "] [" + level + "] [" + tag + "] " + msg

timestamp():
    // 返回 "2026-05-25 14:30:01" 格式字符串
```

**日志级别映射**:

| 事件 | 方法 | 控制台 | 文件 |
|------|------|--------|------|
| 巡逻开始/结束、巡逻点到达/离开、拍照保存 | `info()` | ✓ | ✓ |
| warning 异常、导航超时跳过、低电触发 | `warn()` | ✓ | ✓ |
| critical 异常、导航服务不可达 | `error()` | ✓ | ✓ |

**线程安全**: 所有写操作通过 `std::mutex` 保护，`flush()` 确保崩溃时日志不丢失。

### 2.6 导航客户端 (Nav2ActionClient)

**文件**: `include/patrol_bot/nav2_action_client.hpp`, `src/nav2_action_client.cpp`

```cpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <memory>
#include <atomic>
#include <chrono>

namespace patrol_bot {

// Nav2 NavigateToPose Action 的调用状态
enum class NavResult {
    SUCCESS,      // 导航成功到达
    FAILURE,      // 导航失败 (Nav2 返回 ABORTED 或 CANCELED)
    RUNNING,      // 导航进行中
    NOT_STARTED,  // 尚未发送 goal
    ERROR         // Action 通信异常 (服务端未就绪等)
};

// yaw → quaternion 辅助函数
geometry_msgs::msg::Quaternion yaw_to_quaternion_msg(double yaw);

class Nav2ActionClient {
public:
    Nav2ActionClient(rclcpp::Node* node, const bool mock = false,
                     const std::string& action_name = "/navigate_to_pose");

    // 启用/禁用 mock 模式：导航立即返回成功，无需真实 Nav2
    void set_mock(bool mock);

    // 等待 Action Server 就绪 (阻塞)。mock 模式下立即返回 true
    bool wait_for_server(std::chrono::seconds timeout);

    // 发送导航目标 (在 BT onStart 中调用一次)
    void send_goal(double x, double y, double yaw);

    // 检查导航结果 (在 BT onRunning 中每 tick 调用，非阻塞)
    NavResult check_result();

    // 取消当前导航 (在 BT onHalted 中调用)
    void cancel_goal();

    bool is_navigating() const { return goal_handle_ != nullptr; }

private:
    rclcpp::Node* node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;

    std::atomic<NavResult> result_{NavResult::NOT_STARTED};
    std::atomic<bool> result_ready_{false};
    std::atomic<bool> server_ready_{false};
    bool mock_ = false;
};

}  // namespace patrol_bot
```

**实现伪代码**:

```
yaw_to_quaternion_msg(yaw):
    q = tf2::Quaternion()
    q.setRPY(0, 0, yaw)
    return Quaternion(q.x, q.y, q.z, q.w)

Nav2ActionClient(node, mock, action_name):
    node_ = node
    client_ = rclcpp_action::create_client<NavigateToPose>(node_, action_name)
    if mock: set_mock(true)

set_mock(mock):
    mock_ = mock

wait_for_server(timeout):
    if mock_:
        server_ready_ = true
        return true
    server_ready_ = client_->wait_for_action_server(timeout)
    return server_ready_

send_goal(x, y, yaw):
    // mock 模式: 立即返回 SUCCESS
    if mock_:
        log("[MOCK] Nav2 goal — skipping, returning success")
        result_ = SUCCESS; result_ready_ = true
        return

    if !server_ready_:
        result_ = ERROR; result_ready_ = true
        return

    cancel_goal()  // 清理上一次 goal

    goal_msg = NavigateToPose::Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.header.stamp = node_->now()
    goal_msg.pose.pose.position.x = x
    goal_msg.pose.pose.position.y = y
    goal_msg.pose.pose.orientation = yaw_to_quaternion_msg(yaw)

    options = SendGoalOptions()
    options.goal_response_callback = [this](response) {
        if !response: result_ = ERROR; result_ready_ = true; return
        goal_handle_ = response
    }
    options.result_callback = [this](result) {
        if result.code == SUCCEEDED: result_ = SUCCESS
        else: result_ = FAILURE
        result_ready_ = true
        goal_handle_.reset()
    }

    result_ = NOT_STARTED; result_ready_ = false
    client_->async_send_goal(goal_msg, options)

check_result():
    if result_ready_: return result_
    if goal_handle_ != nullptr: return RUNNING
    return NOT_STARTED

cancel_goal():
    if goal_handle_ != nullptr:
        client_->async_cancel_goal(goal_handle_)
    goal_handle_.reset()
    result_ = NOT_STARTED; result_ready_ = false
```

**Mock 模式说明**: 当 `mock_navigation:=true` 启动参数传入时，`Nav2ActionClient` 构造后会调用 `set_mock(true)`。之后：
- `wait_for_server()` 立即返回 true（不连真实 Action Server）
- `send_goal()` 立即设置 result = SUCCESS（不发送 goal）
- 效果：BT 节点中所有导航操作瞬间成功，可在无仿真环境下完整验证 BT 逻辑

**wait_for_server 位置变更**: `wait_for_server` 从 `patrol_bot_node` 初始化阶段移除，下沉至 `NavigateToWaypoint` 和 `NavigateToCharger` 两个 BT 节点的**构造函数**中执行。使得：
- 每个导航 BT 节点自主负责等待其依赖的 Action Server
- `patrol_bot_node` 初始化不再因 Nav2 未就绪而阻塞/终止
- mock 模式下跳过等待，构造即完成

### 2.7 摄像头缓存 (CameraBuffer)

**文件**: `include/patrol_bot/camera_buffer.hpp`, `src/camera_buffer.cpp`

```cpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <string>

namespace patrol_bot {

class CameraBuffer {
public:
    CameraBuffer(rclcpp::Node* node,
                 const std::string& topic,
                 const std::string& save_dir,
                 const std::string& format);

    // 保存最新帧到文件
    // @param filename 输出文件名 (不含路径和扩展名)
    // @return 成功返回 true，无可用帧返回 false
    bool save_latest(const std::string& filename);

    bool has_frame() const;

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    std::string save_dir_;
    std::string format_;

    cv::Mat latest_frame_;
    bool has_frame_ = false;
    mutable std::mutex mutex_;
};

}  // namespace patrol_bot
```

**实现伪代码**:

```
CameraBuffer(node, topic, save_dir, format):
    save_dir_ = save_dir; format_ = format
    std::filesystem::create_directories(save_dir_)

    qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()
    sub_ = node->create_subscription<Image>(topic, qos,
        std::bind(&CameraBuffer::image_callback, this, _1))

image_callback(msg):
    try:
        frame = cv_bridge::toCvCopy(msg, "bgr8")->image
        lock_guard lock(mutex_)
        latest_frame_ = frame.clone()
        has_frame_ = true
    catch cv_bridge::Exception:
        RCLCPP_ERROR("cv_bridge conversion failed")

has_frame():
    lock_guard lock(mutex_)
    return has_frame_

save_latest(filename):
    lock_guard lock(mutex_)
    if !has_frame_: return false
    filepath = save_dir_ + "/" + filename + "." + format_
    return cv::imwrite(filepath, latest_frame_)
```

**QoS 说明**: 图像订阅使用 `KeepLast(1).best_effort()`——只保留最新帧，采用尽力投递，避免因图像传输延迟导致处理积压。

---

## 3. 行为树节点详细设计

### 3.1 节点注册与工厂

所有自定义 BT 节点在 `patrol_bot_node.cpp` 的 `register_nodes()` 中注册。注册方式分为三类：

**方式一：`registerNodeType<T>` — 无外部依赖的节点，自动构造**

```cpp
// Condition 节点（纯端口读写，无外部对象依赖）
factory.registerNodeType<IsBatteryLow>("IsBatteryLow");
factory.registerNodeType<HasAlarm>("HasAlarm");
factory.registerNodeType<IsAlarmCritical>("IsAlarmCritical");

// StatefulAction 节点（无外部对象依赖）
factory.registerNodeType<WaitAtWaypoint>("WaitAtWaypoint");
```

**方式二：`registerBuilder<T>` — 有外部依赖的节点，通过 lambda 注入**

```cpp
// 注入 PatrolLogger
BT::NodeBuilder builder_restore_ctx =
    [this](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<RestorePatrolContext>(name, config, logger_);
    };
factory.registerBuilder<RestorePatrolContext>("RestorePatrolContext", builder_restore_ctx);

// 注入 PatrolLogger + CameraBuffer
BT::NodeBuilder builder_capture =
    [this](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<CaptureImage>(name, config, camera_buffer_, logger_);
    };
factory.registerBuilder<CaptureImage>("CaptureImage", builder_capture);

// 注入 PatrolLogger + AlarmManager
BT::NodeBuilder builder_alarm =
    [this](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<HandleAlarm>(name, config, alarm_manager_, logger_);
    };
factory.registerBuilder<HandleAlarm>("HandleAlarm", builder_alarm);

// 注入 Nav2ActionClient + PatrolLogger（根据 mock_navigation 参数构造 client）
BT::NodeBuilder builder_nav_wp =
    [this](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<NavigateToWaypoint>(name, config,
            std::make_shared<Nav2ActionClient>(this,
                get_parameter("mock_navigation").as_bool()),
            logger_);
    };
factory.registerBuilder<NavigateToWaypoint>("NavigateToWaypoint", builder_nav_wp);

// NavigateToCharger、SimulateCharging、SetRouteContext、SavePatrolContext 同理
```

**方式三：内联注册 — 极简节点直接在 lambda 中实现**

```cpp
// IsNotStopped / IsNotPaused — 条件节点
factory.registerSimpleCondition("IsNotStopped", [](BT::TreeNode& node) {
    auto bb = node.config().blackboard;
    int state = bb->get<int>("patrol_state");
    return state != static_cast<int>(PatrolState::STOPPED)
                   ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
});

factory.registerSimpleCondition("IsNotPaused", [](BT::TreeNode& node) {
    auto bb = node.config().blackboard;
    int state = bb->get<int>("patrol_state");
    return state != static_cast<int>(PatrolState::PAUSED)
                   ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
});

// AdvanceToNextRoute — 每条路线巡逻完后的索引推进
factory.registerSimpleAction("AdvanceToNextRoute",
    [](const BT::TreeNode& node) {
        auto bb = node.config().blackboard;
        int route_idx = bb->get<int>("current_route_index");
        auto routes = bb->get<std::vector<Route>>("patrol_routes");
        int num_routes = static_cast<int>(routes.size());
        int next_route = (route_idx + 1) % num_routes;
        bb->set<int>("current_route_index", next_route);
        bb->set<int>("current_waypoint_index", 0);
        return BT::NodeStatus::SUCCESS;
    });
```

**设计说明**: `RegisterBuilder` 是 BehaviorTree.CPP v4 提供的自定义构造方式，允许在节点构造时注入任意 C++ 对象（logger、camera_buffer 等），实现依赖注入，避免 BT 节点直接访问全局状态或 ROS2 Node。

### 3.2 Condition 节点实现

所有 Condition 节点继承 `BT::ConditionNode`，通过 **端口 (Ports)** 读取黑板数据，而非直接访问黑板。

#### IsBatteryLow

**节点类型**: `ConditionNode`  
**端口定义**:

```cpp
static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("battery_level"),
        BT::InputPort<double>("low_threshold")
    };
}
```

**逻辑**:

```
tick():
    battery_level = getInput<double>("battery_level").value()   // 缺失则抛异常
    low_threshold = getInput<double>("low_threshold").value()    // 缺失则抛异常
    return (battery_level < low_threshold) ? SUCCESS : FAILURE
```

**XML 用法**: `<IsBatteryLow battery_level="{battery_level}" low_threshold="{low_threshold}"/>`

---

#### HasAlarm

**节点类型**: `ConditionNode`  
**端口定义**:

```cpp
static BT::PortsList providedPorts() {
    return { BT::InputPort<Waypoint>("current_waypoint") };
}
```

**逻辑**:

```
tick():
    wp = getInput<Waypoint>("current_waypoint").value()   // 缺失则抛异常
    return wp.has_alarm ? SUCCESS : FAILURE
```

**XML 用法**: `<HasAlarm current_waypoint="{current_waypoint}"/>`

---

#### IsAlarmCritical

**节点类型**: `ConditionNode`  
**端口定义**:

```cpp
static BT::PortsList providedPorts() {
    return { BT::InputPort<Waypoint>("current_waypoint") };
}
```

**逻辑**:

```
tick():
    wp = getInput<Waypoint>("current_waypoint").value()   // 缺失则抛异常
    if !wp.has_alarm: return FAILURE
    return (wp.alarm.severity == "critical") ? SUCCESS : FAILURE
```

**XML 用法**: `<IsAlarmCritical current_waypoint="{current_waypoint}"/>`

---

### 3.3 Action 节点实现

#### NavigateToWaypoint（StatefulActionNode）⚠

**文件**: `src/bt_nodes/actions/navigate_to_waypoint.cpp`

**构造函数职责**:
1. 保存依赖注入的 `nav2_client_` 和 `logger_`
2. 调用 `nav2_client_->wait_for_server()` 阻塞等待 Nav2 Action Server 就绪（先等 10s，失败则重试 3 次每次 5s，仍失败抛异常）

**端口定义**:

```cpp
static BT::PortsList providedPorts() {
    return {
        BT::InputPort<Route>("current_route"),
        BT::InputPort<double>("nav_timeout"),
        BT::OutputPort<Waypoint>("current_waypoint"),
        BT::BidirectionalPort<int>("current_waypoint_index")
    };
}
```

> `BidirectionalPort<int>` 表示此端口既可读也可写，用于"读取当前序号 → 执行导航 → 递增写回"模式。

**生命周期**:

```
onStart():
    route = getInput<Route>("current_route").value()
    wp_idx = getInput<int>("current_waypoint_index").value()
    timeout = getInput<double>("nav_timeout").value()

    if wp_idx 越界: return FAILURE

    wp = route.waypoints[wp_idx]
    setOutput<Waypoint>("current_waypoint", wp)       // 写入黑板

    nav_timeout_ = timeout
    start_time_ = now()
    waypoint_count_ = route.waypoints.size()
    nav2_client_->send_goal(wp.pose.x, wp.pose.y, wp.pose.yaw)

    logger_->info("Navigating to waypoint " + to_string(wp_idx))
    return RUNNING

onRunning():
    result = nav2_client_->check_result()

    switch result:
        case NOT_STARTED: return RUNNING
        case SUCCESS:
            // 递增 waypoint_index，写入黑板
            next_idx = wp_idx.value() + 1
            setOutput<int>("current_waypoint_index", next_idx)
            logger_->info("Navigation succeeded! next_wp_idx:" + to_string(next_idx))
            return SUCCESS
        case FAILURE:
        case ERROR:
            logger_->warn("Navigation failed")
            return FAILURE

    // 超时检查
    if (now() - start_time_) > nav_timeout_:
        nav2_client_->cancel_goal()
        logger_->warn("Navigation timeout")
        return FAILURE

    return RUNNING

onHalted():
    nav2_client_->cancel_goal()
    logger_->warn("Navigation halted")
```

**关键设计**: waypoint_index 的递增在 NavigateToWaypoint 内部完成（SUCCESS 分支），而非依赖外部 BT 循环的迭代变量。

---

#### NavigateToCharger（StatefulActionNode）⚠

**文件**: `src/bt_nodes/actions/navigate_to_charger.cpp`

**构造函数职责**: 与 NavigateToWaypoint 相同——等待 Nav2 Action Server 就绪。

**端口定义**:

```cpp
static BT::PortsList providedPorts() {
    return { BT::InputPort<Pose2D>("charging_station") };
}
```

**生命周期**:

```
onStart():
    station = getInput<Pose2D>("charging_station").value()
    nav2_client_->send_goal(station.x, station.y, station.yaw)
    logger_->info("Navigating to charger")
    return RUNNING

onRunning():
    result = nav2_client_->check_result()
    switch result:
        case SUCCESS: logger_->info("Reached charger"); return SUCCESS
        case FAILURE/ERROR: logger_->warn("Failed to reach charger"); return FAILURE
        default: return RUNNING

onHalted():
    nav2_client_->cancel_goal()
```

> 与 NavigateToWaypoint 的关键差异: **不设超时**。机器人必须到达充电桩才能充电，不会因耗时过长而放弃。

---

#### WaitAtWaypoint（StatefulActionNode）⚠

**文件**: `src/bt_nodes/actions/wait_at_waypoint.cpp`

**端口定义**:

```cpp
static BT::PortsList providedPorts() {
    return { BT::InputPort<Waypoint>("current_waypoint") };
}
```

**生命周期**:

```
onStart():
    wp = getInput<Waypoint>("current_waypoint").value()
    wait_duration_ = wp.wait_seconds
    if wait_duration_ <= 0: return SUCCESS
    start_time_ = now()
    return RUNNING

onRunning():
    if (now() - start_time_) >= wait_duration_: return SUCCESS
    return RUNNING

onHalted():
    // 无需清理
```

**注册方式**: 无外部依赖，直接 `registerNodeType<WaitAtWaypoint>()`。

---

#### CaptureImage（SyncActionNode）

**文件**: `src/bt_nodes/actions/capture_image.cpp`

**构造函数注入**: `CameraBuffer`, `PatrolLogger`

**端口定义**:

```cpp
static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("current_route_index"),
        BT::InputPort<int>("current_waypoint_index")
    };
}
```

**逻辑**:

```
tick():
    route_idx = getInput<int>("current_route_index").value_or(0)
    wp_idx = getInput<int>("current_waypoint_index").value_or(0)

    // 生成文件名: route{R}_wp{W}_{YYYYMMDD_HHMMSS}
    filename = "route" + to_string(route_idx) + "_wp" + to_string(wp_idx)
             + "_" + format_time(now(), "%Y%m%d_%H%M%S")

    saved = camera_buffer_->save_latest(filename)
    if saved:
        logger_->info("Saved image: " + filename)
    else:
        logger_->warn("No frame available, skipping image capture")
    return SUCCESS  // 降级：无帧不阻塞巡逻
```

---

#### HandleAlarm（SyncActionNode）

**文件**: `src/bt_nodes/actions/handle_alarm.cpp`

**构造函数注入**: `AlarmManager`, `PatrolLogger`

**端口定义**:

```cpp
static BT::PortsList providedPorts() {
    return {
        BT::InputPort<Waypoint>("current_waypoint"),
        BT::OutputPort<int>("patrol_state")
    };
}
```

**逻辑**:

```
tick():
    wp = getInput<Waypoint>("current_waypoint").value()
    if !wp.has_alarm: return SUCCESS   // 无报警直接成功

    alarm_manager_->raise_alarm(wp.alarm.type, wp.alarm.severity, wp.pose.x, wp.pose.y)

    if wp.alarm.severity == "critical":
        setOutput<int>("patrol_state", STATE_PAUSED)    // = 2
        logger_->error("Critical alarm — patrol paused")
    else:
        logger_->warn("Warning alarm logged")
    return SUCCESS
```

---

#### SavePatrolContext（SyncActionNode）

**文件**: `src/bt_nodes/actions/save_patrol_context.cpp`

**构造函数注入**: `PatrolLogger`

**端口定义**:

```cpp
static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("current_route_index"),
        BT::InputPort<int>("current_waypoint_index"),
        BT::OutputPort<int>("saved_route_index"),
        BT::OutputPort<int>("saved_waypoint_idx")
    };
}
```

**逻辑**:

```
tick():
    route_idx = getInput<int>("current_route_index").value()
    wp_idx = getInput<int>("current_waypoint_index").value()

    setOutput<int>("saved_route_index", route_idx)
    setOutput<int>("saved_waypoint_idx", wp_idx)

    logger_->info("Saved context: route=" + to_string(route_idx)
                 + ", waypoint=" + to_string(wp_idx))
    return SUCCESS
```

---

#### RestorePatrolContext（SyncActionNode）

**文件**: `src/bt_nodes/actions/restore_patrol_context.cpp`

**构造函数注入**: `PatrolLogger`

**端口定义**:

```cpp
static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("saved_route_index"),
        BT::InputPort<int>("saved_waypoint_idx"),
        BT::OutputPort<int>("current_route_index"),
        BT::OutputPort<int>("current_waypoint_index"),
        BT::OutputPort<Waypoint>("current_waypoint"),
        BT::InputPort<std::vector<Route>>("patrol_routes")
    };
}
```

**逻辑**:

```
tick():
    saved_route = getInput<int>("saved_route_index")
    saved_wp = getInput<int>("saved_waypoint_idx")
    routes = getInput<vector<Route>>("patrol_routes").value()

    if saved_route && saved_wp && saved_route != -1 && saved_wp != -1:
        route_idx = saved_route.value()
        wp_idx = saved_wp.value()

        setOutput<int>("current_route_index", route_idx)
        setOutput<int>("current_waypoint_index", wp_idx)

        // 验证索引有效后恢复 current_waypoint
        if 索引均在有效范围内:
            setOutput<Waypoint>("current_waypoint", routes[route_idx].waypoints[wp_idx])

        // 清除保存的上下文（直接操作黑板，避免自循环）
        config().blackboard->set<int>("saved_route_index", -1)
        config().blackboard->set<int>("saved_waypoint_idx", -1)

        logger_->info("Restored checkpoint: route=" + str(route_idx) + ", wp=" + str(wp_idx))

    // 无保存上下文则保持 current_route_index=0, current_waypoint_index=0（由黑板初始值决定）
    return SUCCESS
```

> **注意**: `saved_route_index` 和 `saved_waypoint_idx` 的清除使用了直接黑板访问 (`config().blackboard->set`)，而非端口 `setOutput`，以避免端口定义中的 `saved_route_index` 作为 InputPort 无法写入的约束，以及防止与自身输入形成循环。

---

#### SetRouteContext（SyncActionNode）

**文件**: `src/bt_nodes/actions/set_route_context.cpp`

**构造函数注入**: `PatrolLogger`

**设计说明**: 每次外层路线循环迭代开始时调用，从 `patrol_routes` 中取出当前路线写入黑板。同时计算本路线剩余巡逻点数量，驱动内层 `Repeat` 循环。

**端口定义**:

```cpp
static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("current_route_index"),
        BT::InputPort<std::vector<Route>>("patrol_routes"),
        BT::OutputPort<Route>("current_route"),
        BT::OutputPort<int>("waypoint_count"),
        BT::BidirectionalPort<int>("current_waypoint_index")
    };
}
```

**逻辑**:

```
tick():
    route_idx = getInput<int>("current_route_index").value()
    routes = getInput<vector<Route>>("patrol_routes").value()

    if route_idx 越界: return FAILURE

    route = routes[route_idx]
    setOutput<Route>("current_route", route)

    total = route.waypoints.size()

    // 读取 waypoint_index：如果是断点恢复则保持，否则从 0 开始
    wp_input = getInput<int>("current_waypoint_index")
    wp_idx = (wp_input && wp_input >= 0) ? wp_input : 0
    if wp_idx >= total: wp_idx = 0   // 边界保护

    setOutput<int>("waypoint_count", total - wp_idx)     // 剩余数量，驱动 Repeat
    setOutput<int>("current_waypoint_index", wp_idx)

    logger_->info("Set route \"" + route.name + "\", " + to_string(total - wp_idx) + " waypoints")
    return SUCCESS
```

---

#### SimulateCharging（StatefulActionNode）⚠

**文件**: `src/bt_nodes/actions/simulate_charging.cpp`

**构造函数注入**: `PatrolLogger`

**端口定义**:

```cpp
static BT::PortsList providedPorts() {
    return {
        BT::OutputPort<int>("patrol_state"),
        BT::InputPort<double>("recovery_threshold"),
        BT::InputPort<double>("battery_level")
    };
}
```

**生命周期**:

```
onStart():
    recovery_threshold_ = getInput<double>("recovery_threshold").value()
    setOutput<int>("patrol_state", STATE_CHARGING)   // = 3
    logger_->info("Started charging, recovery threshold = " + to_string(recovery_threshold_) + "%")
    return RUNNING

onRunning():
    level = getInput<double>("battery_level").value()
    if level >= recovery_threshold_:
        setOutput<int>("patrol_state", STATE_PATROLLING)   // = 1
        logger_->info("Battery recovered, resuming patrol")
        return SUCCESS
    return RUNNING

onHalted():
    // 无需特殊处理
```

**充电状态转移**: SimulateCharging 的 `onRunning()` 中完成 `CHARGING → PATROLLING` 的状态写回（通过 `setOutput<int>("patrol_state", ...)`）。BatteryModel 订阅 `/patrol/status` 后自动切换为充电模式。

---

### 3.4 行为树 XML 定义

**文件**: `bt_xml/patrol_tree.xml`

```xml
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4">
  <BehaviorTree ID="PatrolBehavior">
    <ReactiveSequence>

      <!-- 顶层状态守卫 -->
      <IsNotStopped/>
      <IsNotPaused/>

      <!-- ReactiveFallback: 充电(高优先) vs 巡逻(低优先) -->
      <ReactiveFallback>

        <!-- ===== 充电子树 (最高优先级) ===== -->
        <Sequence>
          <IsBatteryLow battery_level="{battery_level}"
                        low_threshold="{low_threshold}"/>
          <SavePatrolContext current_route_index="{current_route_index}"
                            current_waypoint_index="{current_waypoint_index}"/>
          <NavigateToCharger charging_station="{charging_station}"/>
          <SimulateCharging recovery_threshold="{recovery_threshold}"
                            battery_level="{battery_level}"
                            patrol_state="{patrol_state}"/>
        </Sequence>

        <!-- ===== 巡逻子流程 ===== -->
        <ReactiveSequence>
          <!-- 断点恢复 (每次进入巡逻子树执行一次) -->
          <RestorePatrolContext saved_route_index="{saved_route_index}"
                               saved_waypoint_idx="{saved_waypoint_idx}"
                               patrol_routes="{patrol_routes}"
                               current_route_index="{current_route_index}"
                               current_waypoint_index="{current_waypoint_index}"
                               current_waypoint="{current_waypoint}"/>

          <!-- 外层循环: 无限循环所有路线 -->
          <Repeat num_cycles="-1">
            <Sequence>
              <!-- 为当前路线设置上下文 -->
              <SetRouteContext current_route_index="{current_route_index}"
                              patrol_routes="{patrol_routes}"
                              current_waypoint_index="{current_waypoint_index}"
                              current_route="{current_route}"
                              waypoint_count="{waypoint_count}"/>

              <!-- 内层循环: 遍历当前路线剩余巡逻点 -->
              <Repeat num_cycles="{waypoint_count}">
                <Sequence>
                  <NavigateToWaypoint current_route="{current_route}"
                                     current_waypoint_index="{current_waypoint_index}"
                                     nav_timeout="{nav_timeout}"
                                     current_waypoint="{current_waypoint}"/>
                  <WaitAtWaypoint current_waypoint="{current_waypoint}"/>
                  <CaptureImage current_route_index="{current_route_index}"
                                current_waypoint_index="{current_waypoint_index}"/>

                  <!-- 异常处理: 用 ForceSuccess 包裹，防止报警失败中断循环 -->
                  <ForceSuccess>
                    <ReactiveFallback>
                      <!-- critical 分支: HasAlarm + IsAlarmCritical → HandleAlarm -->
                      <Sequence>
                        <HasAlarm current_waypoint="{current_waypoint}"/>
                        <IsAlarmCritical current_waypoint="{current_waypoint}"/>
                        <HandleAlarm current_waypoint="{current_waypoint}"
                                    patrol_state="{patrol_state}"/>
                      </Sequence>
                      <!-- warning 分支: HasAlarm → HandleAlarm -->
                      <Sequence>
                        <HasAlarm current_waypoint="{current_waypoint}"/>
                        <HandleAlarm current_waypoint="{current_waypoint}"
                                    patrol_state="{patrol_state}"/>
                      </Sequence>
                    </ReactiveFallback>
                  </ForceSuccess>
                </Sequence>
              </Repeat>

              <!-- 当前路线全部巡逻点完成 → 推进到下一路线 -->
              <AdvanceToNextRoute/>
            </Sequence>
          </Repeat>
        </ReactiveSequence>

      </ReactiveFallback>
    </ReactiveSequence>
  </BehaviorTree>
</root>
```

**BT 节点与 C++ 类对应表**:

| XML 节点 | C++ 类/注册方式 | 类型 | 构造注入 |
|----------|-----------------|------|----------|
| `IsNotStopped` | 内联 `registerSimpleCondition` | Condition | 无 |
| `IsNotPaused` | 内联 `registerSimpleCondition` | Condition | 无 |
| `IsBatteryLow` | `IsBatteryLow` | Condition | 无 |
| `HasAlarm` | `HasAlarm` | Condition | 无 |
| `IsAlarmCritical` | `IsAlarmCritical` | Condition | 无 |
| `RestorePatrolContext` | `RestorePatrolContext` | Action (Sync) | Logger |
| `SetRouteContext` | `SetRouteContext` | Action (Sync) | Logger |
| `NavigateToWaypoint` | `NavigateToWaypoint` | Action (Stateful) | Nav2Client, Logger |
| `WaitAtWaypoint` | `WaitAtWaypoint` | Action (Stateful) | 无 |
| `CaptureImage` | `CaptureImage` | Action (Sync) | CameraBuffer, Logger |
| `HandleAlarm` | `HandleAlarm` | Action (Sync) | AlarmManager, Logger |
| `NavigateToCharger` | `NavigateToCharger` | Action (Stateful) | Nav2Client, Logger |
| `SimulateCharging` | `SimulateCharging` | Action (Stateful) | Logger |
| `SavePatrolContext` | `SavePatrolContext` | Action (Sync) | Logger |
| `AdvanceToNextRoute` | 内联 `registerSimpleAction` | Action (Sync) | 无 |

**数据流与端口**:

```
黑板 (Blackboard) — 持久化键值存储，所有带端口声明的节点通过 InputPort/OutputPort 读写
  ├── patrol_state (int)            → IsNotStopped/IsNotPaused, HandleAlarm, SimulateCharging
  ├── battery_level (double)        → IsBatteryLow, SimulateCharging (由 BatteryModel 写入)
  ├── low_threshold (double)        → IsBatteryLow
  ├── recovery_threshold (double)   → SimulateCharging
  ├── charging_station (Pose2D)     → NavigateToCharger
  ├── nav_timeout (double)          → NavigateToWaypoint
  ├── patrol_routes (vector<Route>) → RestorePatrolContext, SetRouteContext
  ├── current_route_index (int)     → SetRouteContext, CaptureImage, SavePatrolContext, AdvanceToNextRoute
  ├── current_waypoint_index (int)  → NavigateToWaypoint(Bi), SetRouteContext(Bi), SavePatrolContext, CaptureImage
  ├── current_route (Route)         → NavigateToWaypoint
  ├── current_waypoint (Waypoint)   → HasAlarm, IsAlarmCritical, WaitAtWaypoint, HandleAlarm
  ├── waypoint_count (int)          → Repeat num_cycles
  ├── saved_route_index (int)       → RestorePatrolContext(读), SavePatrolContext(写)
  ├── saved_waypoint_idx (int)      → RestorePatrolContext(读), SavePatrolContext(写)
  └── config (Config)               → (初始化时展开为上述各项，此后不再读取)
```

**与旧版设计的关键差异**:
1. **移除 LoadRoutes 节点**: 配置数据（routes、charging_station、threshold 等）在 `patrol_bot_node` 初始化时直接全部写入黑板，不再需要单独的 BT 节点
2. **端口驱动的黑板访问**: 所有 BT 节点通过 `InputPort<T>` / `OutputPort<T>` / `BidirectionalPort<T>` 声明其数据依赖，而非直接操作 `blackboard->get/set`
3. **SetRouteContext + waypoint_count**: 新节点负责计算路线剩余巡逻点数并驱动内层 Repeat 循环
4. **AdvanceToNextRoute**: 内联节点，每条路线完成后递增路线索引
5. **ForceSuccess 包裹报警子流**: 防止报警处理失败导致整个巡逻中断

---

## 4. 系统节点详细设计

### 4.1 patrol_bot_node 生命周期

**文件**: `src/patrol_bot_node.cpp`

```
main():
    1. rclcpp::init(argc, argv)
    2. 创建 PatrolBotNode (继承 rclcpp::Node)
    3. 节点构造: 调用初始化流程 (4.2 节)
    4. 进入 spin: rclcpp::spin(node)
    5. rclcpp::shutdown()
    6. return 0
```

`rclcpp::spin()` 内部以单线程 Executor 模式运行，所有回调（BT tick 定时器、Status 定时器、Battery 回调、Camera 回调、Service 回调）在同一线程串行执行。

### 4.2 初始化流程

```
PatrolBotNode::PatrolBotNode():   // Node("patrol_bot_node")

    # ===== 参数 =====
    share_dir = ament_index_cpp::get_package_share_directory("patrol_bot")
    declare_parameter("config_path", share_dir + "/config/patrol_config.yaml")
    declare_parameter("mock_navigation", false)

    # ===== 配置加载 (fail-fast) =====
    try:
        config_ = ConfigLoader::load(get_parameter("config_path").as_string())
        RCLCPP_INFO("Config loaded")
    catch std::exception:
        RCLCPP_FATAL("Config load failed: %s", e.what())
        throw

    # ===== 核心模块初始化 (顺序独立，可并行构造) =====
    logger_ = make_shared<PatrolLogger>("logs")
    camera_buffer_ = make_shared<CameraBuffer>(
        this, config_.camera.topic, config_.camera.save_directory, config_.camera.image_format)
    battery_model_ = make_shared<BatteryModel>(this, config_.battery)
    alarm_manager_ = make_shared<AlarmManager>(this, *logger_)

    # 注意: Nav2ActionClient 不在此时创建，而是在 register_nodes() 中的
    #       各个 BT 节点 builder lambda 内按需创建（每节点构造一个 client 实例）

    # ===== 黑板初始化 =====
    blackboard_ = BT::Blackboard::create()
    blackboard_->set("config", config_)
    blackboard_->set("patrol_state", static_cast<int>(PatrolState::PATROLLING))
    blackboard_->set("battery_level", config_.battery.initial_level)
    blackboard_->set("saved_route_index", -1)
    blackboard_->set("saved_waypoint_idx", -1)
    blackboard_->set("current_route_index", 0)
    blackboard_->set("current_waypoint_index", 0)
    blackboard_->set("low_threshold", config_.battery.low_threshold)
    blackboard_->set("recovery_threshold", config_.battery.recovery_threshold)
    blackboard_->set("charging_station", config_.charging_station)
    blackboard_->set("nav_timeout", config_.waypoint_timeout)
    blackboard_->set("patrol_routes", config_.routes)

    # ===== BT 工厂注册 =====
    BT::BehaviorTreeFactory factory
    register_nodes(factory)

    # ===== BT 实例化 =====
    bt_xml_path = share_dir + "/bt_xml/patrol_tree.xml"
    tree_ = make_unique<BT::Tree>(factory.createTreeFromFile(bt_xml_path, blackboard_))

    # ===== ROS2 接口 =====
    // Service Servers (4 个)
    start_srv_ = create_service<Trigger>("/patrol/start_patrol", handle_start)
    pause_srv_ = create_service<Trigger>("/patrol/pause_patrol", handle_pause)
    resume_srv_ = create_service<Trigger>("/patrol/resume_patrol", handle_resume)
    stop_srv_ = create_service<Trigger>("/patrol/stop_patrol", handle_stop)

    // Topic Publisher
    status_pub_ = create_publisher<PatrolStatus>("/patrol/status", 10)

    // /patrol/battery 订阅 (BatteryModel 发布, 本节点订阅并写入黑板)
    battery_sub_ = create_subscription<BatteryState>(
        "/patrol/battery", 10, [this](msg) { battery_callback(msg); })

    # ===== 定时器 =====
    bt_timer_ = create_wall_timer(50ms, [this]() { bt_tick(); })
    status_timer_ = create_wall_timer(1s, [this]() { publish_status(); })

    # ===== 启动 BatteryModel =====
    battery_model_->start()

    RCLCPP_INFO("PatrolBot node initialized.")
```

> **设计变更**: `wait_for_server` 从初始化阶段移除。导航 Action Server 的就绪等待改为在 `NavigateToWaypoint` 和 `NavigateToCharger` 两个 BT 节点的构造函数中执行。`patrol_bot_node` 初始化不再依赖 Nav2 就绪。
>
> **初始状态变更**: 黑板中 `patrol_state` 初始化为 `STATE_PATROLLING`（而非 IDLE），意味着节点启动后直接开始巡逻，无需额外发送 start 命令。Service 接口仍完整可用，可随时暂停/恢复/停止。

### 4.3 主循环设计

```
bt_tick():
    state = blackboard_->get<int>("patrol_state")
    if state == STATE_STOPPED: return   // STOPPED 时不再 tick
    tree_->tickExactlyOnce()
    // tickExactlyOnce() 推进行为树一步，内部根据树定义驱动状态流转

battery_callback(msg):
    // BatteryModel 发布 /patrol/battery → 读取 percentage 写入黑板
    level = msg->percentage * 100.0     // BatteryState.percentage 是 0.0-1.0
    blackboard_->set("battery_level", level)

publish_status():
    msg = PatrolStatus()
    msg.state = blackboard_->get<int>("patrol_state")
    msg.current_route_index = blackboard_->get<int>("current_route_index")
    msg.current_waypoint_index = blackboard_->get<int>("current_waypoint_index")
    msg.battery_level = static_cast<float>(blackboard_->get<double>("battery_level"))
    status_pub_->publish(msg)
```

### 4.4 Service 回调实现

```
handle_start(response):
    state = blackboard_->get<int>("patrol_state")
    if state == STATE_IDLE:
        blackboard_->set("patrol_state", STATE_PATROLLING)
        logger_->info("STATE", "Patrol started. IDLE -> PATROLLING")
        response->success = true; response->message = "Patrol started"
    else:
        response->success = false
        response->message = "Cannot start: current state=" + to_string(state)

handle_pause(response):
    state = blackboard_->get<int>("patrol_state")
    if state == STATE_PATROLLING:
        blackboard_->set("patrol_state", STATE_PAUSED)
        logger_->info("STATE", "Patrol paused. PATROLLING -> PAUSED")
        response->success = true; response->message = "Patrol paused"
    else:
        response->success = false
        response->message = "Cannot pause: current state=" + to_string(state)

handle_resume(response):
    state = blackboard_->get<int>("patrol_state")
    if state == STATE_PAUSED:
        blackboard_->set("patrol_state", STATE_PATROLLING)
        logger_->info("STATE", "Patrol resumed. PAUSED -> PATROLLING")
        response->success = true; response->message = "Patrol resumed"
    else:
        response->success = false
        response->message = "Cannot resume: current state=" + to_string(state)

handle_stop(response):
    state = blackboard_->get<int>("patrol_state")
    if state != STATE_STOPPED:
        blackboard_->set("patrol_state", STATE_STOPPED)
        logger_->info("STATE", "Patrol stopped. State " + to_string(state) + " -> STOPPED")
        response->success = true; response->message = "Patrol stopped"
    else:
        response->success = false; response->message = "Already stopped"
```

### 4.5 Topic 发布实现

**状态发布** (1Hz 定时器，见 4.3 节 `publish_status()`):

使用自定义 `PatrolStatus` 消息，字段：
- `uint8 state`：当前巡逻状态 (0-4)
- `int32 current_route_index`：当前路线编号 (-1 表示无)
- `int32 current_waypoint_index`：当前巡逻点编号 (-1 表示无)
- `float32 battery_level`：电量百分比 (0-100)

**报警发布** (事件触发，见 AlarmManager 2.4 节):

使用自定义 `PatrolAlarm` 消息，字段：
- `uint8 severity`：WARNING(0) / CRITICAL(1)
- `string alarm_type`：异常类型
- `float32 x, y`：触发位置
- `builtin_interfaces/Time timestamp`：触发时间

---

## 5. 消息定义

### 5.1 自定义消息文件

**文件**: `patrol_bot_interfaces/msg/PatrolStatus.msg`

```
# 巡逻系统状态枚举
uint8 IDLE=0
uint8 PATROLLING=1
uint8 PAUSED=2
uint8 CHARGING=3
uint8 STOPPED=4

# 状态字段
uint8 state
int32 current_route_index
int32 current_waypoint_index
float32 battery_level
```

**文件**: `patrol_bot_interfaces/msg/PatrolAlarm.msg`

```
# 报警严重性枚举
uint8 WARNING=0
uint8 CRITICAL=1

# 报警字段
uint8 severity
string alarm_type
float32 x
float32 y
builtin_interfaces/Time timestamp
```

> **包结构变更**: 概要设计中消息定义在 `patrol_bot/msg/` 下。详细设计将消息提取为独立包 `patrol_bot_interfaces`，原因：
> - 其他包（如监控面板、后续多机器人调度）可能只需消息依赖，不需行为树/业务逻辑依赖
> - 符合 ROS2 最佳实践（消息包独立编译，减少重建开销）

### 5.2 标准消息使用

| 用途 | 消息类型 | Topic | 包 |
|------|---------|-------|-----|
| 电池状态发布 | `sensor_msgs/msg/BatteryState` | `/patrol/battery` | `sensor_msgs` |
| Service 接口 | `std_srvs/srv/Trigger` | `/patrol/start_patrol` 等 | `std_srvs` |
| Nav2 导航 | `nav2_msgs/action/NavigateToPose` | `/navigate_to_pose` | `nav2_msgs` |
| 摄像头帧 | `sensor_msgs/msg/Image` | 由 YAML 配置 (`/camera/image_raw`) | `sensor_msgs` |

---

## 6. 配置文件详细规格

**文件**: `config/patrol_config.yaml`

```yaml
# 充电桩位姿（必填）
charging_station:
  x: 1.5
  y: 0.8
  yaw: 0.0

# 电池参数（必填）
battery:
  initial_level: 100.0      # 初始电量 %
  low_threshold: 20.0       # 低电阈值 %
  recovery_threshold: 95.0  # 充电恢复阈值 %
  discharge_rate:
    moving: 0.5             # 每秒消耗 %
  charge_rate: 2.0          # 每秒恢复 %

# 摄像头配置（可选，有默认值）
camera:
  topic: "/camera/image_raw"
  image_format: "png"       # png | jpg
  save_directory: "images"

# 巡逻路线（必填，至少 1 条）
patrol_routes:
  - name: "主通道巡检"
    priority: 1             # 越小越优先
    waypoints:
      - x: 2.0
        y: 1.0
        yaw: 0.0
        wait_seconds: 5
      - x: 5.0
        y: 3.0
        yaw: 1.57
        wait_seconds: 3
        alarm_simulate:     # 可选: 模拟异常
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

# Nav2 参数（必填）
nav2:
  waypoint_timeout: 120.0   # 导航超时秒数
```

**字段校验规则**:

| 字段 | 校验 |
|------|------|
| `charging_station.x/y/yaw` | 必填，float，逐字段 try/catch |
| `battery.*` | 必填，`initial_level` ∈ [0, 100]，`low_threshold` < `recovery_threshold` |
| `camera.*` | 可选，缺失时使用默认值 |
| `patrol_routes` | 必填，非空 IsSequence，size > 0 |
| `patrol_routes[].name` | 必填，string |
| `patrol_routes[].waypoints` | 必填，非空 IsSequence，size > 0 |
| `patrol_routes[].waypoints[].x/y/yaw` | 必填，逐字段 try/catch |
| `alarm_simulate.severity` | 可选；如有则必须是 "warning" 或 "critical" |
| `nav2.waypoint_timeout` | 必填，> 0 |

---

## 7. 构建系统设计

### 7.1 CMakeLists.txt

**`patrol_bot_interfaces/CMakeLists.txt`** (消息包):

```cmake
cmake_minimum_required(VERSION 3.8)
project(patrol_bot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PatrolStatus.msg"
  "msg/PatrolAlarm.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

**`patrol_bot/CMakeLists.txt`** (业务逻辑包):

```cmake
cmake_minimum_required(VERSION 3.8)
project(patrol_bot)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(patrol_bot_interfaces REQUIRED)

include_directories(include)

# 核心库 (6 个模块)
add_library(patrol_bot_core STATIC
  src/config_loader.cpp
  src/battery_model.cpp
  src/alarm_manager.cpp
  src/patrol_logger.cpp
  src/nav2_action_client.cpp
  src/camera_buffer.cpp
)

target_include_directories(patrol_bot_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(patrol_bot_core
  rclcpp rclcpp_action std_srvs std_msgs sensor_msgs
  nav2_msgs yaml-cpp OpenCV cv_bridge patrol_bot_interfaces
)

target_link_libraries(patrol_bot_core
  behaviortree_cpp::behaviortree_cpp
  ${OpenCV_LIBS}
)

# 行为树节点库 (12 个节点)
add_library(patrol_bt_nodes STATIC
  src/bt_nodes/conditions/is_battery_low.cpp
  src/bt_nodes/conditions/has_alarm.cpp
  src/bt_nodes/conditions/is_alarm_critical.cpp
  src/bt_nodes/actions/navigate_to_waypoint.cpp
  src/bt_nodes/actions/navigate_to_charger.cpp
  src/bt_nodes/actions/wait_at_waypoint.cpp
  src/bt_nodes/actions/capture_image.cpp
  src/bt_nodes/actions/simulate_charging.cpp
  src/bt_nodes/actions/handle_alarm.cpp
  src/bt_nodes/actions/save_patrol_context.cpp
  src/bt_nodes/actions/restore_patrol_context.cpp
  src/bt_nodes/actions/set_route_context.cpp
)

target_link_libraries(patrol_bt_nodes
  patrol_bot_core
  behaviortree_cpp::behaviortree_cpp
)

# 主可执行文件
add_executable(patrol_bot_node src/patrol_bot_node.cpp)
target_link_libraries(patrol_bot_node
  patrol_bot_core
  patrol_bt_nodes
  behaviortree_cpp::behaviortree_cpp
)

install(TARGETS patrol_bot_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  bt_xml config launch maps models worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### 7.2 package.xml

**`patrol_bot_interfaces/package.xml`**:

```xml
<package format="3">
  <name>patrol_bot_interfaces</name>
  <version>0.1.0</version>
  <description>Custom ROS2 messages for PatrolBot</description>
  <maintainer email="dev@example.com">developer</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <depend>builtin_interfaces</depend>

  <member_of_group>rosidl_interface_packages</member_of_group>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**`patrol_bot/package.xml`**:

```xml
<package format="3">
  <name>patrol_bot</name>
  <version>0.1.0</version>
  <description>Indoor patrol robot system</description>
  <maintainer email="dev@example.com">developer</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>std_srvs</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav2_msgs</depend>
  <depend>behaviortree_cpp</depend>
  <depend>yaml-cpp</depend>
  <depend>libopencv-dev</depend>
  <depend>cv_bridge</depend>
  <depend>patrol_bot_interfaces</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## 8. 线程与并发设计

### 8.1 线程模型

```
┌──────────────────────────────────────────────────┐
│          单线程 Executor (Main Thread)            │
│                                                  │
│  ┌────────────┐  ┌──────────────┐  ┌──────────┐ │
│  │ BT tick    │  │ Status 发布   │  │ Service  │ │
│  │ (50ms)     │  │ (1s 定时器)   │  │ 回调     │ │
│  └────────────┘  └──────────────┘  └──────────┘ │
│  ┌────────────┐  ┌──────────────┐               │
│  │ Battery    │  │ Battery 订阅  │               │
│  │ 定时器(1Hz)│  │ 回调 (1Hz)   │               │
│  └────────────┘  └──────────────┘               │
│  ┌────────────┐                                 │
│  │ Camera     │                                 │
│  │ 订阅回调   │                                 │
│  └────────────┘                                 │
│                                                  │
│  ✅ 所有回调和定时器串行执行，黑板访问天然线程安全 │
└──────────────────────────────────────────────────┘
```

### 8.2 Nav2 回调的原子保护

`Nav2ActionClient` 中的 `result_callback` 和 `goal_response_callback` 虽然是 Nav2 Action 的异步回调，但在单线程 Executor 中也在同一主线程执行。`std::atomic<bool>` 仅作为冗余保护。

### 8.3 CameraBuffer 的互斥保护

`CameraBuffer::image_callback()` 和 `CameraBuffer::save_latest()` 在单线程模型中不会并发调用，`std::mutex` 为防御性编程，确保后续改为多线程 Executor 时无需修改。

### 8.4 PatrolLogger 的互斥保护

同上述，`std::mutex` 为防御性线程安全。

---

## 9. 错误处理与降级策略

### 9.1 启动阶段 (fail-fast)

| 失败场景 | 策略 | 日志级别 |
|---------|------|---------|
| YAML 配置文件不存在 | 抛异常，进程终止 | FATAL |
| YAML 必填字段缺失 | 抛异常，进程终止 | FATAL |
| YAML 字段值域无效 | 抛异常，进程终止 | FATAL |
| BT XML 文件不存在/解析失败 | 抛异常，进程终止 | FATAL |
| 日志文件创建失败 | 抛异常，进程终止 | FATAL |
| Nav2 Action Server 不可达（BT 节点构造时） | 重试后抛异常，进程终止 | FATAL |

### 9.2 运行时降级

| 失败场景 | 策略 | 日志级别 |
|---------|------|---------|
| 导航超时 (> waypoint_timeout) | 跳过当前巡逻点，继续下一个 | WARN |
| Nav2 返回 ABORTED/FAILED | 跳过当前巡逻点，继续下一个 | WARN |
| Camera 帧不可用 | 跳过拍照，继续巡逻 | WARN |
| `/patrol/battery` 断流 | 保留最后已知电量值，不阻塞 | ERROR |
| Service 非法状态请求 | 返回 `success=false` + 说明 | INFO |
| cv_bridge 转换失败 | 丢弃当前帧，不崩溃 | ERROR |

### 9.3 电量边界处理

- `battery_level` 不低于 0.0 (`max(level - rate, 0.0)`)
- `battery_level` 不高于 100.0 (`min(level + rate, 100.0)`)
- 低电判定为 `battery_level < low_threshold`（严格小于）

---

## 10. 测试策略

### 10.1 单元测试

| 模块 | 测试文件 | 覆盖要点 |
|------|---------|---------|
| ConfigLoader | `test/test_config_loader.cpp` | 完整 YAML 解析、缺必填字段、缺可选字段、类型错误、空路线数组、值域校验 |
| BatteryModel | `test/test_battery_model.cpp` | 放电消耗、充电恢复、上下界钳位、充放电模式切换、BatteryState 消息字段 |
| PatrolLogger | `test/test_patrol_logger.cpp` | 文件创建、多级别写入、时间戳格式、flush 行为、线程安全 |
| AlarmManager | `test/test_alarm_manager.cpp` | 消息字段完整、warning/critical 发布、日志输出 |
| CameraBuffer | `test/test_camera_buffer.cpp` | 模拟 Image 消息、cv_bridge 转换、imwrite 输出文件验证、无帧时降级 |
| Nav2ActionClient | `test/test_nav2_action_client.cpp` | mock 模式、send/cancel/check_result 状态机、server 超时处理 |
| BT 节点 | `test/test_bt_nodes.cpp` | Condition 端口读写边界值、StatefulAction 生命周期、halt 取消、超时返回 |

**测试框架**: GoogleTest (`ament_cmake_gtest`)

### 10.2 集成测试

| 场景 | 测试方法 | 验证点 |
|------|---------|--------|
| 启动→巡逻→停止 | launch + Service 调用 | 状态转移正确、BT tick 正常、Topic 有数据 |
| Mock 导航 BT 逻辑 | `mock_navigation:=true` | 无需 Nav2 即可验证完整 BT 流程 |
| 低电→充电→恢复 | 设低 initial_level + 低阈值 | ReactiveFallback 抢占、断点保存/恢复、充电完成切回 |
| Critical 异常→暂停 | YAML 配置 critical 异常 | patrol_state 变 PAUSED、/patrol/alarm 发布 |
| 导航超时跳过 | Nav2 不响应/mock 超时 | 跳过当前点、日志记录、继续下一点 |
| pause/resume | Service 调用序列 | 合法/非法状态拒绝、BT 暂停/恢复 |

**测试环境**: Gazebo Fortress + Nav2 Mock Server，或 `mock_navigation:=true` standalone BT 测试。

---

## 11. 附录：设计决策记录

| # | 决策点 | 方案 | 理由 |
|---|--------|------|------|
| D1 | CHARGING → PATROLLING 转换 | `SimulateCharging.onRunning()` 写 `patrol_state` 输出端口 | 状态与动作内聚，不依赖外部 |
| D2 | Nav2ActionClient 接口 | 非阻塞三步（send_goal / check_result / cancel_goal） | 与 BT StatefulActionNode 生命周期匹配 |
| D3 | Camera 图像获取方式 | 全局缓存 + 黑板端口读取 | 订阅由节点统一管理，拍照不依赖时序 |
| D4 | 依赖等待策略 | Nav2 强依赖(重试)、Camera 弱依赖(降级) | 不影响核心导航功能 |
| D5 | 电池消耗速率 | 仅 moving_rate，PATROLLING 状态统一使用 | 简化模块，实际不区分行进/停留 |
| D6 | BatteryModel 解耦方式 | 发布 `/patrol/battery` (sensor_msgs) | 后续可替换为 Gazebo 电池插件 |
| D7 | 电池模式切换 | 订阅 `/patrol/status` 判断 `state == CHARGING` | BatteryModel 自闭环 |
| D8 | 恢复策略 | 从 saved_waypoint_idx 原地恢复 | 保守，不丢检查点 |
| D9 | Battery Topic 类型 | `sensor_msgs/msg/BatteryState` | Gazebo 电池插件原生支持 |
| D10 | Camera 配置 | YAML camera 段 | 支持替换摄像头 topic |
| D11 | 线程模型 | 单线程 Executor | 当前复杂度不需要并发 |
| D12 | 消息包独立 | `patrol_bot_interfaces` 独立包 | ROS2 最佳实践，减少依赖链 |
| D13 | BT 节点依赖注入 | `RegisterBuilder` + 构造函数注入 | 避免节点直接访问 ROS2 Node/全局状态 |
| D14 | 黑板数据访问 | 端口驱动（InputPort/OutputPort/BidirectionalPort） | BehaviorTree.CPP v4 推荐方式，显式声明数据流 |
| D15 | wait_for_server 位置 | 下沉至 NavigateToWaypoint/NavigateToCharger 构造函数 | patrol_bot_node 初始化解耦，mock 模式简化 |
| D16 | 初始状态 | PATROLLING（直接启动巡逻） | 简化部署流程，Service 接口仍可随时控制 |
| D17 | Mock 导航 | `Nav2ActionClient(node, mock=true)` 构造注入 | 无仿真环境可验证完整 BT 逻辑 |
| D18 | LoadRoutes 移除 | 配置数据在初始化时直接写入黑板 | 消除中间节点，减少 BT 树复杂度 |
| D19 | SetRouteContext + waypoint_count | 新节点，计算剩余巡逻点数驱动 Repeat | 支持断点恢复后的有效迭代次数 |
| D20 | AdvanceToNextRoute | 内联 SimpleAction，路线完成后递增索引 | 轻量实现，职责单一 |
| D21 | 报警 ForceSuccess 包裹 | `ForceSuccess(ReactiveFallback(...))` | 报警处理失败不中断巡逻循环 |

---

*文档版本: v2.0 | 最后更新: 2026-05-29*
