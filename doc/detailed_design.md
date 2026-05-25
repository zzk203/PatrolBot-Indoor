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

本文档为室内巡逻机器人系统的详细设计文档，逐模块、逐节点描述接口契约、内部实现逻辑、状态机和边界条件。开发人员可据此直接编码，无需回溯概要文档。

### 1.2 设计范围

涵盖 `patrol_bot` 包内所有 C++ 源码模块、行为树节点、ROS2 接口、消息定义、构建系统和测试策略。不涉及 Nav2 参数调优、Gazebo 世界建模、Docker 编排配置。

### 1.3 与概要设计的关系

本文档继承概要设计文档（`doc/high-level-design.md`）的架构决策和接口定义，在此基础上细化到函数签名、伪代码和分支逻辑。设计决策记录见附录。

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
    std::string type;       // "smoke_detected" / "temperature_high" 等
    std::string severity;   // "warning" | "critical"
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
    double moving_rate = 0.5;       // 行进每秒消耗 %，停留复用此值
    double charge_rate = 2.0;       // 每秒恢复 %
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

}  // namespace patrol_bot
```

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

**实现伪代码**：

```
Config ConfigLoader::load(yaml_path):
    yaml = YAML::LoadFile(yaml_path)         // 文件不存在 → 抛异常
    config.waypoint_timeout = yaml["nav2"]["waypoint_timeout"].as<double>()

    // 充电站 (必填)
    cs = yaml["charging_station"]
    config.charging_station = {cs["x"], cs["y"], cs["yaw"]}

    // 电池配置 (必填)
    bat = yaml["battery"]
    config.battery = {
        bat["initial_level"],
        bat["low_threshold"],
        bat["recovery_threshold"],
        bat["discharge_rate"]["moving"],     // idle_rate 已移除，不解析
        bat["charge_rate"]
    }

    // 摄像头 (可选，有默认值)
    if yaml["camera"] exists:
        cam = yaml["camera"]
        config.camera.topic = cam["topic"].as<string>(default="/camera/image_raw")
        config.camera.image_format = cam["image_format"].as<string>(default="png")
        config.camera.save_directory = cam["save_directory"].as<string>(default="images")

    // 路线 (必填，至少1条)
    for each route_node in yaml["patrol_routes"]:
        config.routes.push_back(parse_route(route_node))

    return config

Route ConfigLoader::parse_route(node):
    route.name = node["name"].as<string>()
    route.priority = node["priority"].as<int>()
    for each wp_node in node["waypoints"]:
        route.waypoints.push_back(parse_waypoint(wp_node))
    return route

Waypoint ConfigLoader::parse_waypoint(node):
    wp.pose = {node["x"], node["y"], node["yaw"]}
    wp.wait_seconds = node["wait_seconds"].as<double>()
    alarm = parse_alarm(node["alarm_simulate"])  // 可能为空
    if alarm:
        wp.has_alarm = true
        wp.alarm = *alarm
    return wp
```

**依赖**: `yaml-cpp` (>= 0.7.0)。

**错误处理**:
- 文件不存在: `throw std::runtime_error("Config file not found: " + path)`
- 必填字段缺失: `throw std::runtime_error("Missing required field: xxx")`
- 类型不匹配: yaml-cpp 自动抛 `YAML::BadConversion`，外层捕获后包装为 `std::runtime_error`

### 2.3 电池模型 (BatteryModel)

**文件**: `include/patrol_bot/battery_model.hpp`, `src/battery_model.cpp`

#### 设计变更

与概要设计相比，此模块有两处关键调整：
1. **从写黑板改为发布 Topic**: 解耦后可通过 Gazebo 电池插件无缝替换
2. **移除 idle_rate**: PATROLLING 状态下统一使用 `moving_rate` 消耗
3. **通过订阅 `/patrol/status` 感知充放电模式**: 替代直接访问黑板或 `patrol_state`

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

    // /patrol/status 订阅回调: 更新内部充放电模式
    void status_callback(const patrol_bot_interfaces::msg::PatrolStatus::SharedPtr msg);

    rclcpp::Node* node_;
    BatteryConfig config_;
    double level_;

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::Subscription<patrol_bot_interfaces::msg::PatrolStatus>::SharedPtr status_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool is_charging_ = false;   // 从 /patrol/status 的 state 字段推演
};

}  // namespace patrol_bot
```

**实现伪代码**：

```
BatteryModel(node, config):
    level_ = config.initial_level
    config_ = config

    // 发布 /patrol/battery (QoS: reliable, depth=10)
    battery_pub_ = node->create_publisher<sensor_msgs::msg::BatteryState>(
        "/patrol/battery", rclcpp::QoS(10).reliable())

    // 订阅 /patrol/status (QoS: reliable, depth=10)
    status_sub_ = node->create_subscription<PatrolStatus>(
        "/patrol/status", 10,
        [this](msg) { status_callback(msg); })

timer_callback():
    // 根据充放电模式更新电量
    if is_charging_:
        level_ = min(level_ + config_.charge_rate, 100.0)
    else:
        level_ = max(level_ - config_.moving_rate, 0.0)

    // 构造并发布 BatteryState 消息
    msg = sensor_msgs::msg::BatteryState()
    msg.header.stamp = node_->now()
    msg.percentage = level_ / 100.0
    if is_charging_:
        msg.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_CHARGING
    else:
        msg.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_DISCHARGING
    msg.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_GOOD
    battery_pub_->publish(msg)

status_callback(msg):
    // 根据 patrol status 的状态码决定充放电模式
    // CHARGING(3) → 充电; 其他 → 放电
    is_charging_ = (msg.state == PatrolStatus::CHARGING)
```

**生命周期**:
- `start()`: 创建 1Hz 定时器开始运行
- `stop()`: 取消定时器，停止更新
- 无需析构时特殊处理

**可替换性说明**:
后续使用 Gazebo 电池插件时，只需:
1. 移除 `BatteryModel` 实例
2. Gazebo 电池插件配置发布到同名 `/patrol/battery` Topic
3. `patrol_bot_node` 订阅回调无需任何修改

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

**实现伪代码**：

```
AlarmManager(node, logger):
    alarm_pub_ = node->create_publisher<PatrolAlarm>(
        "/patrol/alarm", rclcpp::QoS(10).reliable())
    logger_ = logger

raise_alarm(type, severity, x, y):
    // 构造消息
    msg = PatrolAlarm()
    msg.severity = (severity == "critical") ? PatrolAlarm::CRITICAL
                                             : PatrolAlarm::WARNING
    msg.alarm_type = type
    msg.x = x
    msg.y = y
    msg.timestamp = node_->now()

    // 发布 Topic
    alarm_pub_->publish(msg)

    // 写日志
    if severity == "critical":
        logger_.error("ALARM",
            format("Critical alarm: %s at (%.1f, %.1f)", type, x, y))
    else:
        logger_.warn("ALARM",
            format("Warning alarm: %s at (%.1f, %.1f)", type, x, y))
```

> **注意**: AlarmManager 不负责修改 `patrol_state`（写黑板）。`critical` 异常导致暂停的逻辑由 BT 节点 `HandleAlarm` 执行，以维持模块间职责清晰。

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

**实现伪代码**：

```
PatrolLogger(log_dir):
    // 确保目录存在
    std::filesystem::create_directories(log_dir)

    // 生成文件名: logs/patrol_2026-05-25_14-30-01.log
    now = std::chrono::system_clock::now()
    time_t = std::chrono::system_clock::to_time_t(now)
    tm = std::localtime(&time_t)
    filename = format("patrol_%04d-%02d-%02d_%02d-%02d-%02d.log",
                       tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday,
                       tm.tm_hour, tm.tm_min, tm.tm_sec)
    file_.open(log_dir + "/" + filename)
    if !file_.is_open():
        throw std::runtime_error("Failed to create log file")

~PatrolLogger():
    if file_.is_open():
        file_.close()

info(tag, msg):
    write("INFO", tag, msg)

warn(tag, msg):
    write("WARN", tag, msg)

error(tag, msg):
    write("ERROR", tag, msg)

write(level, tag, msg):
    line = format("[%s] [%s] [%s] %s", timestamp(), level, tag, msg)
    {
        std::lock_guard<std::mutex> lock(mutex_)
        std::cout << line << std::endl     // 控制台输出
        file_ << line << std::endl         // 文件输出
        file_.flush()                      // 立即落盘，防崩溃丢失
    }

timestamp():
    // 返回 "2026-05-25 14:30:01" 格式字符串
```

**日志级别映射**:

| 事件 | 方法 | 控制台 | 文件 |
|------|------|--------|------|
| 巡逻开始/结束、充电开始/结束、巡逻点到达/离开 | `info()` | ✓ | ✓ |
| warning 异常、导航超时跳过、低电触发 | `warn()` | ✓ | ✓ |
| critical 异常 | `error()` | ✓ | ✓ |

**线程安全**: 所有写操作通过 `std::mutex` 保护，`flush()` 确保崩溃时日志不丢失。

### 2.6 导航客户端 (Nav2ActionClient)

**文件**: `include/patrol_bot/nav2_action_client.hpp`, `src/nav2_action_client.cpp`

#### 设计变更

概设中描述为"阻塞等待"，详细设计改为**非阻塞三步接口**，与 BT `StatefulActionNode` 的 `onStart/onRunning/onHalted` 生命周期配合：

```cpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <atomic>

namespace patrol_bot {

// Nav2 NavigateToPose Action 的调用状态
enum class NavResult {
    SUCCESS,      // 导航成功到达
    FAILURE,      // 导航失败 (Nav2 返回 ABORTED 或 CANCELED)
    RUNNING,      // 导航进行中
    NOT_STARTED,  // 尚未发送 goal
    ERROR         // Action 通信异常 (服务端未就绪等)
};

class Nav2ActionClient {
public:
    Nav2ActionClient(rclcpp::Node* node,
                     const std::string& action_name = "/navigate_to_pose");

    // 等待 Action Server 就绪 (阻塞，最多 timeout 秒)
    // 返回 false 表示超时
    bool wait_for_server(std::chrono::seconds timeout);

    // 发送导航目标 (在 BT onStart 中调用一次)
    void send_goal(double x, double y, double yaw);

    // 检查导航结果 (在 BT onRunning 中每 tick 调用，非阻塞)
    // BT Action 节点根据返回值决定 RUNNING / SUCCESS / FAILURE
    NavResult check_result();

    // 取消当前导航 (在 BT onHalted 中调用)
    void cancel_goal();

    // 是否正在导航中
    bool is_navigating() const { return goal_handle_ != nullptr; }

private:
    rclcpp::Node* node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;

    std::atomic<NavResult> result_{NavResult::NOT_STARTED};
    std::atomic<bool> result_ready_{false};
    std::atomic<bool> server_ready_{false};
};

}  // namespace patrol_bot
```

**实现伪代码**：

```
Nav2ActionClient(node, action_name):
    node_ = node
    client_ = rclcpp_action::create_client<NavigateToPose>(node_, action_name)

wait_for_server(timeout):
    server_ready_ = client_->wait_for_action_server(timeout)
    return server_ready_

send_goal(x, y, yaw):
    if !server_ready_:
        result_ = NavResult::ERROR
        result_ready_ = true
        return

    result_ = NavResult::NOT_STARTED
    result_ready_ = false

    // 构造目标
    goal_msg = NavigateToPose::Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.header.stamp = node_->now()
    goal_msg.pose.pose.position.x = x
    goal_msg.pose.pose.position.y = y
    // yaw → quaternion
    goal_msg.pose.pose.orientation = quaternion_from_yaw(yaw)

    // 发送异步 goal，注册回调
    send_goal_options = rclcpp_action::Client<...>::SendGoalOptions()

    send_goal_options.result_callback =
        [this](result) {
            if result.code == rclcpp_action::ResultCode::SUCCEEDED:
                result_ = NavResult::SUCCESS
            else:
                result_ = NavResult::FAILURE
            result_ready_ = true
        }

    send_goal_options.goal_response_callback =
        [this](response) {
            if !response->accepted:
                result_ = NavResult::ERROR
                result_ready_ = true
        }

    client_->async_send_goal(goal_msg, send_goal_options)

check_result():
    if result_ready_:
        return result_
    return NavResult::RUNNING

cancel_goal():
    if goal_handle_:
        client_->async_cancel_goal(goal_handle_)
        goal_handle_ = nullptr
    result_ = NavResult::NOT_STARTED
    result_ready_ = false
```

**注意**: `send_goal` 和 `check_result` 必须在同一个线程中调用（单线程 executor 保证），回调修改 `result_` / `result_ready_` 使用 `std::atomic` 以确保可见性。

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

    // 保存最新帧到文件 (由 CaptureImage BT 节点调用)
    // @param filename 输出文件名 (不含路径和扩展名)
    // @return 成功返回 true，无可用帧返回 false
    bool save_latest(const std::string& filename);

    // 是否有可用帧
    bool has_frame() const;

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    std::string save_dir_;
    std::string format_;  // "png" / "jpg"

    cv::Mat latest_frame_;
    bool has_frame_ = false;
    std::mutex mutex_;
};

}  // namespace patrol_bot
```

**实现伪代码**：

```
CameraBuffer(node, topic, save_dir, format):
    save_dir_ = save_dir
    format_ = format
    std::filesystem::create_directories(save_dir)
    sub_ = node->create_subscription<sensor_msgs::msg::Image>(
        topic, 10, [this](msg) { image_callback(msg); })

image_callback(msg):
    // cv_bridge: ROS Image → cv::Mat (BGR8)
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8")
    {
        std::lock_guard lock(mutex_)
        latest_frame_ = cv_ptr->image.clone()
        has_frame_ = true
    }

has_frame():
    std::lock_guard lock(mutex_)
    return has_frame_

save_latest(filename):
    std::lock_guard lock(mutex_)
    if !has_frame_:
        return false
    path = format("%s/%s.%s", save_dir_, filename, format_)
    cv::imwrite(path, latest_frame_)
    return true
```

---

## 3. 行为树节点详细设计

### 3.1 节点注册与工厂

所有自定义 BT 节点通过 `BehaviorTree.CPP` 的 `RegisterSimpleAction` / `RegisterSimpleCondition` 或自定义 Builder 注册。

**注册代码** (在 `patrol_bot_node.cpp` 中):

```cpp
#include "behaviortree_cpp/bt_factory.h"

void register_nodes(BT::BehaviorTreeFactory& factory) {
    // Condition 节点
    factory.registerSimpleCondition("IsBatteryLow",
        std::bind(&is_battery_low_fn, std::ref(blackboard)));
    factory.registerSimpleCondition("HasAlarm",
        std::bind(&has_alarm_fn, std::ref(blackboard)));
    factory.registerSimpleCondition("IsAlarmCritical",
        std::bind(&is_alarm_critical_fn, std::ref(blackboard)));

    // Action 节点 (使用 StatefulActionNode 的自定义 builder)
    factory.registerBuilder<NavigateToWaypoint>(
        "NavigateToWaypoint", builder_fn);
    // ... 其余 Action 节点类似
}
```

### 3.2 Condition 节点实现

#### IsBatteryLow

**节点类型**: `ConditionNode` (SimpleCondition)

**逻辑**:
```
tick():
    battery = blackboard->get<double>("battery_level")
    threshold = blackboard->get<double>("low_threshold")
    return (battery < threshold) ? SUCCESS : FAILURE
```

**读取黑板**: `battery_level`, `low_threshold`  
**写黑板**: 无  
**副作用**: 无

---

#### HasAlarm

**节点类型**: `ConditionNode` (SimpleCondition)

**逻辑**:
```
tick():
    wp = blackboard->get<Waypoint>("current_waypoint")
    return wp.has_alarm ? SUCCESS : FAILURE
```

**读取黑板**: `current_waypoint.has_alarm`  
**写黑板**: 无

---

#### IsAlarmCritical

**节点类型**: `ConditionNode` (SimpleCondition)

**逻辑**:
```
tick():
    wp = blackboard->get<Waypoint>("current_waypoint")
    if !wp.has_alarm:
        return FAILURE
    return (wp.alarm.severity == "critical") ? SUCCESS : FAILURE
```

**读取黑板**: `current_waypoint.alarm.severity`  
**写黑板**: 无

---

### 3.3 Action 节点实现

#### NavigateToWaypoint

**节点类型**: `StatefulActionNode` ⚠

**生命周期**:

```
onStart():
    wp = blackboard->get<Waypoint>("current_waypoint")
    timeout = blackboard->get<double>("nav_timeout")

    nav_client->send_goal(wp.pose.x, wp.pose.y, wp.pose.yaw)
    start_time_ = node_->now()
    return RUNNING

onRunning():
    elapsed = (node_->now() - start_time_).seconds()

    result = nav_client->check_result()
    if result == NavResult::SUCCESS:
        return SUCCESS
    if result == NavResult::FAILURE:
        return FAILURE
    if elapsed > timeout:
        nav_client->cancel_goal()
        logger.warn("NAV_TIMEOUT",
            format("Timeout at route=%d waypoint=%d",
                   current_route_idx, current_waypoint_idx))
        return FAILURE

    return RUNNING

onHalted():
    nav_client->cancel_goal()
```

**读取黑板**: `current_waypoint.pose`, `nav_timeout`  
**写黑板**: 无  
**错误处理**:
- Nav2 返回 ABORTED → FAILURE (BT 跳过本点)
- 超时 → FAILURE (记录 WARN 日志)
- 被 halt (低电/暂停抢占) → 取消 goal，无日志

---

#### NavigateToCharger

**节点类型**: `StatefulActionNode` ⚠

**逻辑与 NavigateToWaypoint 相同，但目标坐标为黑板 `charging_station`**:

```
onStart():
    cs = blackboard->get<Pose2D>("charging_station")
    nav_client->send_goal(cs.x, cs.y, cs.yaw)
    start_time_ = node_->now()
    return RUNNING
```

> **注意**: 到充电桩的导航不设超时。机器人必须到达充电桩才能充电。

---

#### WaitAtWaypoint

**节点类型**: `StatefulActionNode` ⚠

**生命周期**:

```
onStart():
    wp = blackboard->get<Waypoint>("current_waypoint")
    wait_seconds_ = wp.wait_seconds
    if wait_seconds_ <= 0:
        return SUCCESS
    start_time_ = node_->now()
    return RUNNING

onRunning():
    if (node_->now() - start_time_).seconds() >= wait_seconds_:
        return SUCCESS
    return RUNNING

onHalted():
    // 被抢占中断，无需清理
```

**读取黑板**: `current_waypoint.wait_seconds`  
**写黑板**: 无

---

#### CaptureImage

**节点类型**: `SyncActionNode`

**逻辑**:

```
tick():
    route_idx = blackboard->get<int>("current_route_index")
    wp_idx = blackboard->get<int>("current_waypoint_index")

    timestamp = format_time(now(), "%Y%m%d_%H%M%S")
    filename = format("route%d_wp%d_%s", route_idx, wp_idx, timestamp)

    // camera_buffer 在构造时注入
    if !camera_buffer->has_frame():
        logger.warn("CAMERA", "No camera frame available, skip capture")
        return SUCCESS  // 降级：拍照失败不阻塞巡逻

    camera_buffer->save_latest(filename)
    logger.info("CAPTURE",
        format("Image saved: %s", filename))
    return SUCCESS
```

**读取黑板**: `current_route_index`, `current_waypoint_index`  
**写黑板**: 无  
**降级处理**: 无摄像头帧时记录 WARN 并跳过，不影响巡逻。

---

#### SimulateCharging

**节点类型**: `StatefulActionNode` ⚠

**生命周期**:

```
onStart():
    // 设置 patol_state = CHARGING
    blackboard->set("patrol_state", STATE_CHARGING)

    recovery_threshold = blackboard->get<double>("recovery_threshold")
    start_time_ = node_->now()
    return RUNNING

onRunning():
    battery_level = blackboard->get<double>("battery_level")
    if battery_level >= recovery_threshold:
        // 充电完成 → 切回 PATROLLING 状态
        blackboard->set("patrol_state", STATE_PATROLLING)
        logger.info("CHARGING",
            format("Charging complete at %.1f%%. Resuming patrol.", battery_level))
        return SUCCESS
    return RUNNING

onHalted():
    // 被 stop 指令中断，不做特殊处理
```

**读取黑板**: `battery_level`, `recovery_threshold`  
**写黑板**: `patrol_state` (CHARGING → PATROLLING)  
**状态转移触发**: 此节点负责 CHARGING → PATROLLING 的状态转移（回答用户确认的设计决策）。

---

#### HandleAlarm

**节点类型**: `SyncActionNode`

**逻辑**:

```
tick():
    wp = blackboard->get<Waypoint>("current_waypoint")
    severity = wp.alarm.severity
    type = wp.alarm.type

    alarm_manager->raise_alarm(type, severity, wp.pose.x, wp.pose.y)

    if severity == "critical":
        // critical 异常：暂停巡逻
        blackboard->set("patrol_state", STATE_PAUSED)
        logger.error("ALARM",
            format("Critical alarm! Patrol paused. Type=%s Pos=(%.1f,%.1f)",
                   type, wp.pose.x, wp.pose.y))
    else:
        // warning 异常：仅记录，继续巡逻
        logger.warn("ALARM",
            format("Warning alarm logged. Type=%s Pos=(%.1f,%.1f)",
                   type, wp.pose.x, wp.pose.y))

    return SUCCESS
```

**读取黑板**: `current_waypoint`  
**写黑板**: `patrol_state` (仅 critical 时写 PAUSED)

---

#### SavePatrolContext

**节点类型**: `SyncActionNode`

**逻辑**:

```
tick():
    route_idx = blackboard->get<int>("current_route_index")
    wp_idx = blackboard->get<int>("current_waypoint_index")

    blackboard->set("saved_route_index", route_idx)
    blackboard->set("saved_waypoint_idx", wp_idx)

    logger.info("CHARGING",
        format("Context saved: route=%d, waypoint=%d", route_idx, wp_idx))
    return SUCCESS
```

**读取黑板**: `current_route_index`, `current_waypoint_index`  
**写黑板**: `saved_route_index`, `saved_waypoint_idx`

---

#### RestorePatrolContext

**节点类型**: `SyncActionNode`

**设计说明**: 此节点为新增节点（概设未单独列出），与 `LoadRoutes` 分离职责——LoadRoutes 仅加载原始路线数据，RestorePatrolContext 负责断点恢复。

**逻辑**:

```
tick():
    saved_route = blackboard->get<int>("saved_route_index")
    saved_wp = blackboard->get<int>("saved_waypoint_idx")

    if saved_route >= 0 && saved_wp >= 0:
        // 有保存的断点 → 恢复上下文
        blackboard->set("current_route_index", saved_route)
        blackboard->set("current_waypoint_index", saved_wp)

        routes = blackboard->get<vector<Route>>("patrol_routes")
        blackboard->set("current_waypoint", routes[saved_route].waypoints[saved_wp])

        logger.info("ROUTE",
            format("Resuming route %d from waypoint %d", saved_route, saved_wp))

        // 清除保存的上下文
        blackboard->set("saved_route_index", -1)
        blackboard->set("saved_waypoint_idx", -1)
    else:
        // 首次启动，从路线 0 巡逻点 0 开始
        blackboard->set("current_route_index", 0)
        blackboard->set("current_waypoint_index", 0)

    return SUCCESS
```

**读取黑板**: `saved_route_index`, `saved_waypoint_idx`, `patrol_routes`  
**写黑板**: `current_route_index`, `current_waypoint_index`, `current_waypoint`

---

#### LoadRoutes

**节点类型**: `SyncActionNode`

**逻辑**:

```
tick():
    config = blackboard->get<Config>("config")
    routes = config.routes

    blackboard->set("patrol_routes", routes)
    blackboard->set("charging_station", config.charging_station)

    // 电池/导航参数写入黑板
    blackboard->set("low_threshold", config.battery.low_threshold)
    blackboard->set("recovery_threshold", config.battery.recovery_threshold)
    blackboard->set("nav_timeout", config.waypoint_timeout)

    logger.info("ROUTE",
        format("Loaded %zu patrol route(s)", routes.size()))
    return SUCCESS
```

**读取黑板**: `config`  
**写黑板**: `patrol_routes`, `charging_station`, `low_threshold`, `recovery_threshold`, `nav_timeout`

---

#### SetRouteContext

**节点类型**: `SyncActionNode`

**逻辑**:

```
tick():
    // 此节点在路线 loop 的每次迭代开始时执行
    // 当前路线和巡逻点编号已由外部 loop 或 RestorePatrolContext 设置
    route_idx = blackboard->get<int>("current_route_index")
    routes = blackboard->get<vector<Route>>("patrol_routes")

    blackboard->set("current_route", routes[route_idx])

    logger.info("ROUTE",
        format("Starting route: \"%s\" (index: %d)",
               routes[route_idx].name, route_idx))
    return SUCCESS
```

**读取黑板**: `current_route_index`, `patrol_routes`  
**写黑板**: `current_route`

---

### 3.4 行为树 XML 定义

**文件**: `bt_xml/patrol_tree.xml`

```xml
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4">
  <BehaviorTree ID="PatrolBehavior">
    <ReactiveSequence>

      <!-- 顶层 Condition: 非 STOPPED 状态 -->
      <Condition ID="IsNotStopped"/>

      <!-- 顶层 Condition: 非 PAUSED 状态 -->
      <Condition ID="IsNotPaused"/>

      <!-- 抢占式回退: 充电 (高优先级) vs 巡逻 (低优先级) -->
      <ReactiveFallback>

        <!-- ===== 充电子树 (优先级最高) ===== -->
        <Sequence>
          <Condition ID="IsBatteryLow"/>
          <Action ID="SavePatrolContext"/>
          <Action ID="NavigateToCharger"/>
          <Action ID="SimulateCharging"/>
        </Sequence>

        <!-- ===== 巡逻子流程 ===== -->
        <ReactiveSequence>
          <Action ID="LoadRoutes"/>
          <Action ID="RestorePatrolContext"/>

          <!-- 外层 Loop: 循环执行所有路线 -->
          <Loop num_cycles="-1">
            <Sequence>
              <Action ID="SetRouteContext"/>

              <!-- 内层 Loop: 循环路线内所有巡逻点 -->
              <Loop num_cycles="-1">
                <Sequence>
                  <Action ID="NavigateToWaypoint"/>
                  <Action ID="WaitAtWaypoint"/>
                  <Action ID="CaptureImage"/>

                  <!-- 异常处理: critical 或 warning -->
                  <ReactiveFallback>
                    <Sequence>
                      <Condition ID="HasAlarm"/>
                      <Condition ID="IsAlarmCritical"/>
                      <Action ID="HandleAlarm"/>
                    </Sequence>
                    <Sequence>
                      <Condition ID="HasAlarm"/>
                      <Action ID="HandleAlarm"/>
                    </Sequence>
                  </ReactiveFallback>

                  <!-- 巡逻点索引 +1 (隐式由 Loop 控制) -->
                </Sequence>
              </Loop>
            </Sequence>
          </Loop>
        </ReactiveSequence>

      </ReactiveFallback>
    </ReactiveSequence>
  </BehaviorTree>
</root>
```

**BT 节点与 C++ 类对应表**:

| XML 节点 | C++ 类/注册名 | 类型 |
|----------|---------------|------|
| `IsNotStopped` | 内联 SimpleCondition | Condition |
| `IsNotPaused` | 内联 SimpleCondition | Condition |
| `IsBatteryLow` | `is_battery_low_fn` | Condition |
| `HasAlarm` | `has_alarm_fn` | Condition |
| `IsAlarmCritical` | `is_alarm_critical_fn` | Condition |
| `LoadRoutes` | `LoadRoutes` | Action (Sync) |
| `RestorePatrolContext` | `RestorePatrolContext` | Action (Sync) |
| `SetRouteContext` | `SetRouteContext` | Action (Sync) |
| `NavigateToWaypoint` | `NavigateToWaypoint` | Action (Stateful) |
| `WaitAtWaypoint` | `WaitAtWaypoint` | Action (Stateful) |
| `CaptureImage` | `CaptureImage` | Action (Sync) |
| `NavigateToCharger` | `NavigateToCharger` | Action (Stateful) |
| `SimulateCharging` | `SimulateCharging` | Action (Stateful) |
| `HandleAlarm` | `HandleAlarm` | Action (Sync) |
| `SavePatrolContext` | `SavePatrolContext` | Action (Sync) |

**IsNotStopped / IsNotPaused 内联实现**:

```cpp
// 在 patrol_bot_node.cpp 中注册
factory.registerSimpleCondition("IsNotStopped", [&](BT::TreeNode& node) {
    auto bb = node.config().blackboard;
    return bb->get<int>("patrol_state") != 4 ? BT::NodeStatus::SUCCESS
                                              : BT::NodeStatus::FAILURE;
});

factory.registerSimpleCondition("IsNotPaused", [&](BT::TreeNode& node) {
    auto bb = node.config().blackboard;
    return bb->get<int>("patrol_state") != 2 ? BT::NodeStatus::SUCCESS
                                              : BT::NodeStatus::FAILURE;
});
```

---

## 4. 系统节点详细设计

### 4.1 patrol_bot_node 生命周期

**文件**: `src/patrol_bot_node.cpp`

```
main():
    1. rclcpp::init()
    2. 创建 PatrolBotNode (继承 rclcpp::Node)
    3. 节点构造: 调用初始化流程 (4.2 节)
    4. 进入 spin: rclcpp::spin(node)
    5. rclcpp::shutdown()
```

### 4.2 初始化流程

```
PatrolBotNode::PatrolBotNode():
    # --- 阶段 1: 参数声明 ---
    declare_parameter("config_path", "config/patrol_config.yaml")

    # --- 阶段 2: 配置加载 (fail-fast) ---
    try:
        config_ = ConfigLoader::load(get_parameter("config_path").as_string())
    catch std::runtime_error:
        RCLCPP_FATAL("Config load failed: %s", e.what())
        throw  // 终止进程

    # --- 阶段 3: 核心模块初始化 (顺序独立) ---
    logger_ = std::make_shared<PatrolLogger>("logs")
    camera_buffer_ = std::make_shared<CameraBuffer>(
        this, config_.camera.topic,
        config_.camera.save_directory, config_.camera.image_format)
    battery_model_ = std::make_shared<BatteryModel>(this, config_.battery)
    alarm_manager_ = std::make_shared<AlarmManager>(this, *logger_)
    nav_client_ = std::make_shared<Nav2ActionClient>(this)

    # --- 阶段 4: 等待 Nav2 Action Server ---
    if !nav_client_->wait_for_server(10s):
        RCLCPP_WARN("Nav2 action server not ready, retrying...")
        for i in 0..2:
            if nav_client_->wait_for_server(5s):
                break
        if !nav_client_->server_ready():
            RCLCPP_FATAL("Nav2 action server unavailable after retries")
            throw std::runtime_error("Nav2 not ready")

    # --- 阶段 5: 黑板初始化 ---
    blackboard_ = BT::Blackboard::create()
    blackboard_->set("config", config_)
    blackboard_->set("patrol_state", STATE_IDLE)           // 0
    blackboard_->set("battery_level", config_.battery.initial_level)
    blackboard_->set("saved_route_index", -1)
    blackboard_->set("saved_waypoint_idx", -1)
    blackboard_->set("current_route_index", 0)
    blackboard_->set("current_waypoint_index", 0)

    # --- 阶段 6: BT 工厂注册 ---
    factory_ = BT::BehaviorTreeFactory()
    register_nodes(factory_)

    # --- 阶段 7: BT 实例化 ---
    tree_ = factory_.createTreeFromFile("bt_xml/patrol_tree.xml", blackboard_)

    # --- 阶段 8: ROS2 接口 ---
    // Service Server
    start_srv_ = create_service<Trigger>(
        "/patrol/start_patrol", [this](req, resp) { handle_start(req, resp); })
    pause_srv_ = create_service<Trigger>(
        "/patrol/pause_patrol", [this](req, resp) { handle_pause(req, resp); })
    resume_srv_ = create_service<Trigger>(
        "/patrol/resume_patrol", [this](req, resp) { handle_resume(req, resp); })
    stop_srv_ = create_service<Trigger>(
        "/patrol/stop_patrol", [this](req, resp) { handle_stop(req, resp); })

    // Topic Publisher
    status_pub_ = create_publisher<PatrolStatus>("/patrol/status", 10)

    // /patrol/battery 订阅: BatteryModel 发布, 本节点订阅并写入黑板
    battery_sub_ = create_subscription<BatteryState>(
        "/patrol/battery", 10,
        [this](msg) { battery_callback(msg); })

    # --- 阶段 9: 定时器 ---
    // BT tick 定时器: 50ms
    bt_timer_ = create_wall_timer(50ms, [this]() { bt_tick(); })

    // Status 发布定时器: 1s
    status_timer_ = create_wall_timer(1s, [this]() { publish_status(); })

    # --- 阶段 10: 启动 BatteryModel ---
    battery_model_->start()

    RCLCPP_INFO("PatrolBot node initialized. Waiting for start command...")
```

### 4.3 主循环设计

```
bt_tick():
    state = blackboard_->get<int>("patrol_state")
    if state == STATE_STOPPED:
        return  // STOPPED 时不再 tick

    status = tree_->tickRoot()
    // tickRoot() 内部根据行为树定义驱动状态流转

battery_callback(msg):
    // BatteryModel 发布 /patrol/battery → 写入黑板
    level = msg.percentage * 100.0  // BatteryState 是 0.0-1.0
    blackboard_->set("battery_level", level)

publish_status():
    msg = PatrolStatus()
    msg.state = blackboard_->get<int>("patrol_state")
    msg.current_route_index = blackboard_->get<int>("current_route_index")
    msg.current_waypoint_index = blackboard_->get<int>("current_waypoint_index")
    msg.battery_level = blackboard_->get<double>("battery_level")
    status_pub_->publish(msg)
```

### 4.4 Service 回调实现

```
handle_start(req, resp):
    state = blackboard_->get<int>("patrol_state")
    if state == STATE_IDLE:
        blackboard_->set("patrol_state", STATE_PATROLLING)
        logger_->info("STATE", "Patrol started. IDLE -> PATROLLING")
        resp->success = true
        resp->message = "Patrol started"
    else:
        resp->success = false
        resp->message = format("Cannot start: current state=%d", state)

handle_pause(req, resp):
    state = blackboard_->get<int>("patrol_state")
    if state == STATE_PATROLLING:
        blackboard_->set("patrol_state", STATE_PAUSED)
        logger_->info("STATE", "Patrol paused. PATROLLING -> PAUSED")
        resp->success = true
        resp->message = "Patrol paused"
    else:
        resp->success = false
        resp->message = format("Cannot pause: current state=%d", state)

handle_resume(req, resp):
    state = blackboard_->get<int>("patrol_state")
    if state == STATE_PAUSED:
        blackboard_->set("patrol_state", STATE_PATROLLING)
        logger_->info("STATE", "Patrol resumed. PAUSED -> PATROLLING")
        resp->success = true
        resp->message = "Patrol resumed"
    else:
        resp->success = false
        resp->message = format("Cannot resume: current state=%d", state)

handle_stop(req, resp):
    state = blackboard_->get<int>("patrol_state")
    if state != STATE_STOPPED:
        blackboard_->set("patrol_state", STATE_STOPPED)
        logger_->info("STATE",
            format("Patrol stopped. State %d -> STOPPED", state))
        resp->success = true
        resp->message = "Patrol stopped"
    else:
        resp->success = false
        resp->message = "Already stopped"
```

### 4.5 Topic 发布实现

**状态发布** (1Hz 定时器，见 4.3 节 `publish_status()`):

使用自定义 `PatrolStatus` 消息，字段：
- `uint8 state`：当前巡逻状态 (0-4)
- `int32 current_route_index`：当前路线编号 (-1 表示无)
- `int32 current_waypoint_index`：当前巡逻点编号 (-1 表示无)
- `float32 battery_level`：电量百分比

**报警发布** (事件触发，见 AlarmManager 2.4 节):

使用自定义 `PatrolAlarm` 消息，字段：
- `uint8 severity`：WARNING(0) / CRITICAL(1)
- `string alarm_type`：异常类型
- `float32 x, y`：触发位置
- `Time timestamp`：触发时间

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
# 充电桩位姿 (必填)
charging_station:
  x: 1.5
  y: 0.8
  yaw: 0.0

# 电池参数 (必填)
battery:
  initial_level: 100.0      # 初始电量 %
  low_threshold: 20.0       # 低电阈值 %
  recovery_threshold: 95.0  # 充电恢复阈值 %
  discharge_rate:
    moving: 0.5             # 每秒消耗 % (停留复用此值)
  charge_rate: 2.0          # 每秒恢复 %

# 摄像头配置 (可选，有默认值)
camera:
  topic: "/camera/image_raw"
  image_format: "png"       # png | jpg
  save_directory: "images"

# 巡逻路线 (必填，至少 1 条)
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

# Nav2 参数 (必填)
nav2:
  waypoint_timeout: 120.0   # 导航超时秒数
```

**字段校验规则**:

| 字段 | 校验 |
|------|------|
| `charging_station.x/y/yaw` | 必填，float |
| `battery.*` | 必填，`initial_level` ∈ [0, 100]，`low_threshold` < `recovery_threshold` |
| `camera.*` | 可选，缺失时使用默认值 |
| `patrol_routes` | 必填，非空数组 |
| `patrol_routes[].waypoints` | 必填，非空数组 |
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

# 核心库
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

# 行为树节点库
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
  src/bt_nodes/actions/load_routes.cpp
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
  bt_xml
  config
  launch
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
│  │ 定时器     │  │ 回调 (1Hz)   │               │
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
| Nav2 Action Server 不可达 (3 次重试后) | 抛异常，进程终止 | FATAL |
| 日志文件创建失败 | 抛异常，进程终止 | FATAL |
| BT XML 文件不存在/解析失败 | 抛异常，进程终止 | FATAL |

### 9.2 运行时降级

| 失败场景 | 策略 | 日志级别 |
|---------|------|---------|
| 导航超时 (> waypoint_timeout) | 跳过当前巡逻点，继续下一个 | WARN |
| Nav2 返回 ABORTED/FAILED | 跳过当前巡逻点，继续下一个 | WARN |
| Camera 帧不可用 | 跳过拍照，继续巡逻 | WARN |
| `/patrol/battery` 断流 | 保留最后已知电量值，不阻塞 | ERROR |
| Service 非法状态请求 | 返回 `success=false` + 说明 | INFO |

### 9.3 电量边界处理

- `battery_level` 不低于 0.0（`max(level - rate, 0.0)`）
- `battery_level` 不高于 100.0（`min(level + rate, 100.0)`）
- 低电阈值为 `battery_level < low_threshold`（严格小于）

---

## 10. 测试策略

### 10.1 单元测试

| 模块 | 测试文件 | 覆盖要点 |
|------|---------|---------|
| ConfigLoader | `test_config_loader.cpp` | 完整 YAML 解析、缺必填字段、缺可选字段、类型错误、空路线数组 |
| BatteryModel | `test_battery_model.cpp` | 放电消耗、充电恢复、上下界钳位、充放电模式切换、BatteryState 消息字段 |
| PatrolLogger | `test_patrol_logger.cpp` | 文件创建、多级别写入、时间戳格式、flush 行为、线程安全 |
| AlarmManager | `test_alarm_manager.cpp` | 消息字段完整、warning 不暂停、critical 发布 |
| CameraBuffer | `test_camera_buffer.cpp` | 模拟 Image 消息、cv_bridge 转换、imwrite 输出文件验证、无帧时降级 |
| Nav2ActionClient | `test_nav2_action_client.cpp` | mock Action Server、send/cancel/check_result 状态机 |
| BT Condition 节点 | `test_bt_conditions.cpp` | IsBatteryLow/IsAlarmCritical 边界值、黑板变量缺失行为 |
| BT Action 节点 | `test_bt_actions.cpp` | StatefulActionNode 生命周期、halt 取消、超时返回 |

**测试框架**: GoogleTest (`ament_cmake_gtest`)

### 10.2 集成测试

| 场景 | 测试方法 | 验证点 |
|------|---------|--------|
| 启动→巡逻→停止 | launch + Service 调用 | 状态转移正确、BT tick 正常、Topic 有数据 |
| 低电→充电→恢复 | 设低 initial_level + 低阈值 | ReactiveFallback 抢占、断点保存/恢复、充电完成切回 |
| Critical 异常→暂停 | YAML 配置 critical 异常 | patrol_state 变 PAUSED、/patrol/alarm 发布 |
| 导航超时跳过 | Nav2 不响应/mock 超时 | 跳过当前点、日志记录、继续下一点 |
| pause/resume | Service 调用序列 | 合法/非法状态拒绝、BT 暂停/恢复 |

**测试环境**: Gazebo Fortress + Nav2 Mock Server 或 CI 中 standalone BT 测试。

---

## 11. 附录：设计决策记录

| # | 决策点 | 方案 | 理由 |
|---|--------|------|------|
| D1 | CHARGING → PATROLLING 转换 | SimulateCharging.onSuccess() 写入 | 状态与动作内聚，不依赖外部 |
| D2 | Nav2ActionClient 阻塞 vs 非阻塞 | 非阻塞三步接口 | 与 BT StatefulActionNode 生命周期匹配 |
| D3 | Camera 图像获取方式 | 全局缓存 + 黑板读取 | 订阅由节点统一管理，拍照不依赖时序 |
| D4 | 依赖等待策略 | Nav2 强依赖(重试)、Camera 弱依赖(降级) | 不影响核心导航功能 |
| D5 | 电池消耗速率 | 移除 idle_rate，PATROLLING 统一 moving_rate | 简化模块，实际电池不需区分 |
| D6 | BatteryModel 解耦方式 | 发布 /patrol/battery (sensor_msgs) | 后续可替换为 Gazebo 电池插件 |
| D7 | 电池模式切换 | 订阅 /patrol/status 判断 CHARGING | BatteryModel 自闭环 |
| D8 | 恢复策略 | 从 saved_waypoint_idx 原地重走 | 保守，不丢检查点 |
| D9 | Battery Topic 类型 | sensor_msgs/msg/BatteryState | Gazebo 电池插件原生支持 |
| D10 | Camera 配置 | YAML camera 段 | 支持替换摄像头 topic |
| D11 | 线程模型 | 单线程 Executor | 当前复杂度不需要并发 |
| D12 | 消息包独立 | patrol_bot_interfaces 独立包 | ROS2 最佳实践，减少依赖链 |
| D13 | LoadRoutes 与 RestorePatrolContext 分离 | 两个独立 Action 节点 | 加载 vs 恢复职责清晰 |

---

*文档版本: v1.0 | 最后更新: 2026-05-25*
