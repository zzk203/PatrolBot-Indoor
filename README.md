# PatrolBot-Indoor

室内巡逻机器人系统，基于 ROS2 Humble + BehaviorTree.CPP v4 + Nav2 + Gazebo Fortress。

机器人按 YAML 配置的预设路线自动巡逻，具备低电自动回充断点恢复、异常检测分级报警、巡逻点拍照存档等完整巡检能力。

---

## 技术栈

| 组件 | 选型 |
|------|------|
| 操作系统 | Ubuntu 22.04 |
| ROS2 | Humble |
| 行为树 | BehaviorTree.CPP v4 |
| 仿真引擎 | Gazebo Fortress (Ignition) |
| 导航框架 | Nav2 (Navigation2) |
| 机器人 | TurtleBot3 差分驱动 |
| C++ 标准 | C++17 |
| 构建 | colcon + ament_cmake |
| 容器化 | Docker |
| 配置 | YAML (yaml-cpp) |
| 测试 | GoogleTest (ament_cmake_gtest) |

---

## 功能

- **YAML 配置驱动**：巡逻路线、巡逻点、充电桩坐标、电池参数、报警模拟等全部通过 `config/patrol_config.yaml` 配置，修改后无需重新编译
- **多路线循环巡逻**：按 YAML 定义顺序依次执行多条路线，循环往复
- **行为树编排**：关键决策逻辑（低电判断、异常分级、路线切换）通过 BehaviorTree.CPP v4 编排，底层动作为叶子节点实现
- **低电自动回充**：电量低于阈值时中断巡逻，保存断点，自动导航回充电桩，充电完成后原地恢复巡逻
- **异常分级报警**：`warning`（仅记录日志，继续巡逻）和 `critical`（暂停巡逻，等待人工介入）两级
- **巡逻点拍照存档**：到达每个巡逻点后通过摄像头拍照，文件名含路线编号、巡逻点编号和时间戳
- **动态避障与超时跳过**：Nav2 局部规划器自动绕行，超时未到达则跳过当前点继续下一个
- **ROS2 Service 启停控制**：start / pause / resume / stop 四个 Service 接口 + ROS2 Topic 状态/报警发布
- **Mock 导航模式**：`mock_navigation:=true` 可跳过 Nav2/Gazebo，直接验证行为树逻辑
- **日志双通道输出**：终端实时 + 文件持久，日志按日期自动分文件

---

## 目录结构

```
PatrolBot-Indoor/
├── README.md
├── doc/
│   ├── proposal.md                  # 功能需求文档
│   ├── high_level_design.md         # 概要设计
│   ├── detailed_design.md           # 详细设计（含接口契约、伪代码）
│   ├── prompt.md                    # 开发提示词
│   └── tasks/                       # 各模块子任务清单
│       ├── 01-interfaces-and-types.md
│       ├── 02-config-loader.md
│       ├── 03-patrol-logger.md
│       ├── 04-battery-model.md
│       ├── 05-alarm-manager.md
│       ├── 06-nav2-client.md
│       ├── 07-camera-buffer.md
│       ├── 08-bt-nodes.md
│       ├── 09-patrol-node.md
│       ├── 10-build-and-launch.md
│       └── progress.md              # 模块完成状态
├── docker/
│   ├── Dockerfile                   # ROS2 Humble 镜像
│   ├── docker-compose.yml
│   └── dev.sh                       # 容器启动脚本
├── docs_v1/                         # v1 版里程碑检查清单（历史文档）
└── patrol_robot_ws/
    └── src/
        ├── patrol_bot_interfaces/   # 自定义 ROS2 消息包
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   └── msg/
        │       ├── PatrolStatus.msg   # 巡逻状态消息（含 IDLE/PATROLLING/PAUSED/CHARGING/STOPPED）
        │       └── PatrolAlarm.msg    # 报警消息（WARNING/CRITICAL + 位置 + 时间戳）
        └── patrol_bot/              # 主业务包
            ├── include/patrol_bot/  # 头文件
            │   ├── patrol_types.hpp     # 内部数据结构（Pose2D, Waypoint, Route, Config 等）
            │   ├── config_loader.hpp    # YAML 配置加载器
            │   ├── battery_model.hpp    # 电池模拟（发布 sensor_msgs/BatteryState）
            │   ├── alarm_manager.hpp    # 报警管理器
            │   ├── patrol_logger.hpp    # 日志（双通道：终端 + 文件）
            │   ├── nav2_action_client.hpp  # Nav2 导航客户端（非阻塞 + mock 支持）
            │   ├── camera_buffer.hpp    # 摄像头帧缓存
            │   └── bt_nodes.hpp         # 所有 BT 节点类声明
            ├── src/
            │   ├── patrol_bot_node.cpp  # 主节点（初始化、BT tick、Service/Topic 接口）
            │   ├── config_loader.cpp
            │   ├── battery_model.cpp
            │   ├── alarm_manager.cpp
            │   ├── patrol_logger.cpp
            │   ├── nav2_action_client.cpp
            │   ├── camera_buffer.cpp
            │   └── bt_nodes/            # 行为树节点实现
            │       ├── conditions/
            │       │   ├── is_battery_low.cpp
            │       │   ├── has_alarm.cpp
            │       │   └── is_alarm_critical.cpp
            │       └── actions/
            │           ├── navigate_to_waypoint.cpp
            │           ├── navigate_to_charger.cpp
            │           ├── wait_at_waypoint.cpp
            │           ├── capture_image.cpp
            │           ├── handle_alarm.cpp
            │           ├── save_patrol_context.cpp
            │           ├── restore_patrol_context.cpp
            │           ├── set_route_context.cpp
            │           └── simulate_charging.cpp
            ├── bt_xml/
            │   └── patrol_tree.xml       # 行为树 XML 定义
            ├── config/
            │   ├── patrol_config.yaml    # 巡逻配置
            │   └── nav2_params.yaml      # Nav2 参数
            ├── launch/
            │   ├── patrol_bot.launch.py          # 巡逻节点 + Nav2 导航栈
            │   └── patrol_simulation.launch.py   # 完整仿真（Gazebo + 桥接 + 导航 + 巡逻）
            ├── maps/
            │   └── office.yaml           # 预建地图
            ├── scripts/                  # 控制脚本
            │   ├── start_patrol.sh
            │   ├── pause_patrol.sh
            │   ├── resume_patrol.sh
            │   ├── stop_patrol.sh
            │   ├── odom_republisher.py   # 里程计帧名转换辅助
            │   ├── odom_bridge.py
            │   └── pointcloud_to_scan.py
            ├── models/                   # 机器人 URDF 模型
            ├── worlds/                   # Gazebo 仿真世界
            └── test/                     # 单元测试（GoogleTest）
                ├── test_config_loader.cpp
                ├── test_battery_model.cpp
                ├── test_patrol_logger.cpp
                ├── test_alarm_manager.cpp
                ├── test_nav2_action_client.cpp
                ├── test_camera_buffer.cpp
                └── test_bt_nodes.cpp
```

---

## 架构概览

```
┌─────────────────────────────────────────────────────────────────┐
│                    patrol_bot_node (50ms tick)                  │
│                                                                 │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │        BehaviorTree.CPP (patrol_tree.xml)                 │  │
│  │                                                           │  │
│  │  ReactiveSequence                                         │  │
│  │  ├─ IsNotStopped / IsNotPaused  (顶层状态守卫)             │  │
│  │  └─ ReactiveFallback                                       │  │
│  │     ├─ [高优先] 低电 → 保存上下文 → 导航充电桩 → 模拟充电  │  │
│  │     └─ [低优先] 路线循环 → 巡逻点导航 → 等待 → 拍照 → 报警  │  │
│  │                                                           │  │
│  │  黑板 (Blackboard): patrol_state, battery_level,          │  │
│  │     current_route/wapoint, patrol_routes, config...       │  │
│  └───────────────────────────────────────────────────────────┘  │
│                                                                 │
│  状态机: IDLE(0) → PATROLLING(1) → PAUSED(2) → CHARGING(3) →  │
│          STOPPED(4)                                             │
│                                                                 │
│  ROS2 接口:                                                     │
│  /patrol/start_patrol | pause_patrol | resume_patrol |         │
│  stop_patrol  (std_srvs/Trigger)                               │
│  /patrol/status (PatrolStatus, 1Hz)                            │
│  /patrol/alarm  (PatrolAlarm, 事件触发)                         │
│  /patrol/battery (sensor_msgs/BatteryState, 1Hz)               │
└─────────────────────────────────────────────────────────────────┘
```

### 数据结构

```
Config
├── charging_station: Pose2D         # 充电桩坐标
├── battery: BatteryConfig           # 电池参数
├── routes: vector<Route>            # 巡逻路线列表
├── waypoint_timeout: double         # 导航超时
└── camera: CameraConfig             # 摄像头配置

Route                               Waypoint
├── name: string                    ├── pose: Pose2D
├── priority: int                   ├── wait_seconds: double
└── waypoints: vector<Waypoint>     ├── has_alarm: bool
                                    └── alarm: AlarmConfig
```

---

## 构建

### 环境准备（Docker）

```bash
# 构建并进入开发容器
bash docker/dev.sh
```

### 编译

```bash
cd /root/patrol_robot_ws
source /opt/ros/humble/setup.bash

# 编译所有包
colcon build --packages-select patrol_bot_interfaces patrol_bot
# 本地调试用，编译所有包，install的部分使用软链接（改动相关文件时不用重新编译）,如果脚本没有执行权限需要先赋权
# chmod +x src/patrol_bot/scripts/*.py
# chmod +x src/patrol_bot/scripts/*.sh
colcon build --symlink-install --packages-select patrol_bot_interfaces patrol_bot

# 或编译全部 workspace
colcon build
```

### 测试

```bash
# 运行全部单元测试
colcon test --packages-select patrol_bot --return-code-on-test-failure

# 查看测试结果
colcon test-result --all

# 单独运行某个测试
./build/patrol_bot/test_config_loader
./build/patrol_bot/test_bt_nodes
```

### 静态分析

```bash
cppcheck --enable=all --suppress=missingIncludeSystem \
    src/patrol_bot/src/ src/patrol_bot/include/

find src/patrol_bot -name '*.cpp' | xargs clang-tidy \
    -p build/patrol_bot
```

---

## 运行

```bash
source install/setup.bash

# === 模式 1: Mock 导航（无需仿真/Nav2，直接验证 BT 逻辑） ===
ros2 launch patrol_bot patrol_bot.launch.py mock_navigation:=true enable_navigation:=false

# === 模式 2: 仅巡逻节点 + Nav2（需要传感器数据） ===
ros2 launch patrol_bot patrol_bot.launch.py

# === 模式 3: 完整仿真（Gazebo + 桥接 + Nav2 + Patrol） ===
ros2 launch patrol_bot patrol_simulation.launch.py

# === 模式 4: 仿真 + 关闭导航（仅 Gazebo + patrol_bot_node，方便调试） ===
ros2 launch patrol_bot patrol_simulation.launch.py enable_navigation:=false mock_navigation:=true
```

### 控制命令

```bash
# 启停控制
ros2 service call /patrol/start_patrol std_srvs/srv/Trigger "{}"
ros2 service call /patrol/pause_patrol std_srvs/srv/Trigger "{}"
ros2 service call /patrol/resume_patrol std_srvs/srv/Trigger "{}"
ros2 service call /patrol/stop_patrol std_srvs/srv/Trigger "{}"

# 或使用快捷脚本
bash src/patrol_bot/scripts/start_patrol.sh
bash src/patrol_bot/scripts/pause_patrol.sh
bash src/patrol_bot/scripts/resume_patrol.sh
bash src/patrol_bot/scripts/stop_patrol.sh
```

### 监控话题

```bash
# 查看巡逻状态（state, route_index, waypoint_index, battery_level）
ros2 topic echo /patrol/status

# 查看电池状态
ros2 topic echo /patrol/battery

# 查看报警事件
ros2 topic echo /patrol/alarm

# 查看机器人坐标
ros2 run tf2_ros tf2_echo map base_link

# 查看话题发布者/订阅者
ros2 topic info /patrol/status -v
ros2 topic info /patrol/battery -v
ros2 topic info /patrol/alarm -v
```

---

## 配置文件

编辑 `patrol_robot_ws/src/patrol_bot/config/patrol_config.yaml` 可调整以下内容，无需修改代码：

| 配置段 | 说明 |
|--------|------|
| `charging_station` | 充电桩位姿 (x, y, yaw) |
| `battery` | 初始电量、低电阈值、恢复阈值、放电/充电速率 |
| `camera` | 摄像头 topic、图片格式、保存目录（可选） |
| `patrol_routes` | 巡逻路线列表，每条含优先级、路线名、巡逻点序列 |
| `patrol_routes[].waypoints` | 巡逻点 (x, y, yaw, wait_seconds, alarm_simulate) |
| `nav2` | waypoint_timeout（导航超时秒数） |

**巡逻点 `alarm_simulate` 示例**:

```yaml
alarm_simulate:
  type: temperature_high    # 异常类型
  severity: critical         # warning 或 critical
```

---

## Mock 导航模式

`mock_navigation:=true` 参数启用后 `Nav2ActionClient` 跳过真实的 Nav2 Action Server 通信，`send_goal()` 立即返回 SUCCESS。用途：

- **行为树逻辑调试**：不依赖 Gazebo/Nav2/传感器即可快速迭代验证 BT 流程
- **离线开发**：在无仿真环境的机器上验证业务逻辑
- **CI 测试**：纯逻辑测试无需启动仿真环境

Mock 模式通过 `Nav2ActionClient(node, mock=true)` 构造函数注入，BT 节点在 `register_nodes()` 中根据参数创建 mock 或真实 client。

### 关闭导航服务

`enable_navigation:=false` 参数用于跳过 Nav2 导航栈（map_server / amcl / planner_server / controller_server / behavior_server / bt_navigator / lifecycle_manager）的启动：

- 与 `mock_navigation` 独立控制：`enable_navigation` 控制是否**启动** Nav2 节点，`mock_navigation` 控制 patrol_bot_node **内部**是否真的调用 Nav2
- `enable_navigation:=false mock_navigation:=true`：最简启动，仅 patrol_bot_node + robot_state_publisher，适合纯 BT 逻辑验证
- `enable_navigation:=true mock_navigation:=true`：启动 Nav2 栈但 patrol 不调用，适合单独调试 Nav2
- `enable_navigation:=true mock_navigation:=false`（默认）：完整导航模式

---

## 设计文档

| 文档 | 说明 |
|------|------|
| [doc/proposal.md](doc/proposal.md) | 功能需求文档（FR-01 ~ FR-16） |
| [doc/high_level_design.md](doc/high_level_design.md) | 概要设计（架构、模块划分、接口） |
| [doc/detailed_design.md](doc/detailed_design.md) | 详细设计（接口契约、伪代码、状态机、错误处理） |
| [doc/tasks/](doc/tasks/) | 各模块子任务清单与完成进度 |

---

## 关键设计决策

| # | 决策 | 说明 |
|---|------|------|
| 线程模型 | 单线程 Executor | 所有回调和 BT tick 串行，黑板访问天然安全 |
| Nav2 接口 | 非阻塞三步（send_goal / check_result / cancel_goal） | 与 BT StatefulActionNode 生命周期匹配 |
| 电池解耦 | 发布 `sensor_msgs/BatteryState` 到 `/patrol/battery` | 后续可无缝替换为 Gazebo 电池插件 |
| 消息包独立 | `patrol_bot_interfaces` 独立编译 | ROS2 最佳实践，减少重建开销 |
| 充电状态转移 | `SimulateCharging` 节点内完成 CHARGING→PATROLLING | 状态与动作内聚，不依赖外部 |
| 停等耗电 | PATROLLING 状态统一使用 moving_rate | 简化电池模型，实际不区分行进/停留 |
| 恢复策略 | 从断点（saved_waypoint_idx）原地恢复 | 保守设计，不丢检查点 |
| wait_for_server | 下沉至 BT 节点内部 | 导航服务就绪等待由 BT 节点按需执行 |
