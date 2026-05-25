# PatrolBot-Indoor 开发 Prompt

## 目标

在 Gazebo Fortress 仿真环境中实现室内巡逻机器人系统，机器人按 YAML 配置的固定路线自动巡逻，具备低电回充、异常检测报警、日志记录等完整巡检能力。

## 技术栈

| 组件       | 版本/选型                |
| ---------- | ------------------------ |
| 操作系统   | Ubuntu 22.04             |
| ROS2       | Humble                   |
| 行为树     | BehaviorTree.CPP v4      |
| 仿真引擎   | Gazebo Fortress (Ignition) |
| 导航框架   | Nav2 (Navigation2)       |
| 机器人     | TurtleBot3 差分驱动      |
| 传感器     | LiDAR + Camera           |
| 构建       | colcon + ament_cmake     |
| C++ 标准   | C++17                    |

## 工作空间与目录结构

- 工作空间根目录：`/home/zzk/PatrolBot-Indoor/patrol_robot_ws/`
- 主包路径：`/home/zzk/PatrolBot-Indoor/patrol_robot_ws/src/patrol_bot`
- 消息包路径：`/home/zzk/PatrolBot-Indoor/patrol_robot_ws/src/patrol_bot_interfaces`
- 设计文档：`/home/zzk/PatrolBot-Indoor/doc/`
- docker配置的ros2环境启动：`/home/zzk/PatrolBot-Indoor/docker/`

```
patrol_robot_ws/src/
├── patrol_bot_interfaces/       # 消息包
│   ├── msg/
│   │   ├── PatrolStatus.msg
│   │   └── PatrolAlarm.msg
│   ├── CMakeLists.txt
│   └── package.xml
└── patrol_bot/                  # 主业务包
    ├── include/patrol_bot/
    │   ├── patrol_types.hpp     # 内部数据结构 + PatrolState 枚举
    │   ├── config_loader.hpp
    │   ├── battery_model.hpp
    │   ├── alarm_manager.hpp
    │   ├── patrol_logger.hpp
    │   ├── nav2_action_client.hpp
    │   └── camera_buffer.hpp
    ├── src/
    │   ├── config_loader.cpp
    │   ├── battery_model.cpp
    │   ├── alarm_manager.cpp
    │   ├── patrol_logger.cpp
    │   ├── nav2_action_client.cpp
    │   ├── camera_buffer.cpp
    │   ├── patrol_bot_node.cpp    # 主节点入口
    │   └── bt_nodes/
    │       ├── conditions/
    │       │   ├── is_battery_low.cpp
    │       │   ├── has_alarm.cpp
    │       │   └── is_alarm_critical.cpp
    │       └── actions/
    │           ├── load_routes.cpp
    │           ├── restore_patrol_context.cpp
    │           ├── set_route_context.cpp
    │           ├── capture_image.cpp
    │           ├── handle_alarm.cpp
    │           ├── save_patrol_context.cpp
    │           ├── navigate_to_waypoint.cpp
    │           ├── navigate_to_charger.cpp
    │           ├── wait_at_waypoint.cpp
    │           └── simulate_charging.cpp
    ├── bt_xml/
    │   └── patrol_tree.xml
    ├── config/
    │   ├── patrol_config.yaml
    │   └── nav2_params.yaml
    ├── launch/
    │   ├── patrol_bot.launch.py
    │   └── patrol_simulation.launch.py
    ├── scripts/
    │   ├── start_patrol.sh
    │   ├── pause_patrol.sh
    │   ├── resume_patrol.sh
    │   └── stop_patrol.sh
    ├── test/
    │   ├── test_config_loader.cpp
    │   ├── test_battery_model.cpp
    │   ├── test_patrol_logger.cpp
    │   ├── test_alarm_manager.cpp
    │   ├── test_camera_buffer.cpp
    │   ├── test_nav2_action_client.cpp
    │   ├── test_bt_conditions.cpp
    │   └── test_bt_actions.cpp
    ├── CMakeLists.txt
    └── package.xml
```

## 执行模式

- **主 agent（你）**：负责跟踪整体进度，按顺序调度子 agent 完成各个模块，子agent完成后你需要进行验收（实际运行测试）和审计，若验收不通过你需要子agent修改。每完成一个模块后更新 `doc/tasks/progress.md` 中的状态。
- **子 agent（executor）**：负责实现单个模块的完整代码（头文件 + 源文件 + 单元测试），完成后验证编译通过且测试通过。
- **整个过程中不会有人工参与**。遇到错误时子 agent 需自行修复，主 agent 负责重试和调度。

## 模块实现顺序（必须严格按此顺序）

模块间有依赖关系，不可并行或跳过：

| 序号 | 模块                     | 依赖                       | 任务文档                                       |
| ---- | ------------------------ | -------------------------- | ---------------------------------------------- |
| 01   | 消息包与内部数据结构     | 无                         | `doc/tasks/01-interfaces-and-types.md`         |
| 02   | 配置加载器 (ConfigLoader)| 01                         | `doc/tasks/02-config-loader.md`                |
| 03   | 巡逻日志 (PatrolLogger)  | 无（仅标准库依赖）         | `doc/tasks/03-patrol-logger.md`                |
| 04   | 电池模型 (BatteryModel)  | 01                         | `doc/tasks/04-battery-model.md`                |
| 05   | 报警管理器 (AlarmManager)| 01, 03                     | `doc/tasks/05-alarm-manager.md`                |
| 06   | 导航客户端 (Nav2ActionClient) | 无（仅 ROS2 标准类型） | `doc/tasks/06-nav2-client.md`                  |
| 07   | 摄像头缓存 (CameraBuffer)| 无（仅 ROS2 + OpenCV）     | `doc/tasks/07-camera-buffer.md`                |
| 08   | 行为树节点 (BT Nodes)    | 01, 02, 03, 04, 05, 06, 07 | `doc/tasks/08-bt-nodes.md`                     |
| 09   | 系统主节点 (patrol_bot_node) | 01-08（所有模块）     | `doc/tasks/09-patrol-node.md`                  |
| 10   | 构建系统、启动脚本与测试 | 01-09（所有模块）          | `doc/tasks/10-build-and-launch.md`             |

## 设计参考

- 需求文档：`doc/proposal.md`
- 详细设计：`doc/detailed_design.md`
- 各模块的子任务清单：`doc/tasks/` 目录下对应文件

## 关键设计决策（勿自行更改）

| #  | 决策                               | 约束                                                       |
| -- | ---------------------------------- | ---------------------------------------------------------- |
| D1 | PatrolState 枚举定义位置           | `include/patrol_bot/patrol_types.hpp` 中 `enum class PatrolState` |
| D2 | BT 节点依赖注入方式                 | 通过 `BT::NodeBuilder` 构造函数注入模块 shared_ptr，黑板只存数据不存指针 |
| D3 | Nav2ActionClient 接口              | 非阻塞三步接口（send_goal / check_result / cancel_goal）   |
| D4 | 电池消耗速率                       | PATROLLING 和停留统一使用 moving_rate，无 idle_rate        |
| D5 | BatteryModel 模式感知              | 订阅 `/patrol/status` Topic，`state==CHARGING(3)` 时充电   |
| D6 | 消息包                             | `patrol_bot_interfaces` 独立包                             |
| D7 | CHARGING→PATROLLING 状态转移       | 由 `SimulateCharging` 节点的 `onRunning` 中检测电量恢复后写入黑板 |
| D8 | 断点恢复策略                       | 从保存的 waypoint 原地重走，不清除已完成的巡逻记录         |
| D9 | 线程模型                           | 单线程 Executor，所有回调串行执行，黑板访问天然线程安全    |
| D10| LoadRoutes 与 RestorePatrolContext | 分离为两个独立 Action 节点，分别负责加载和断点恢复         |

## 代码质量要求

### C++ 部分

- **编译器警告**：`-Wall -Wextra -Werror`，零警告通过
- **clang-tidy**：开启所有 recommended checks，零警告
- **cppcheck**：开启 `--enable=all --suppress=missingIncludeSystem`，零错误

### Python 部分（仅 launch 脚本）

- **ruff**：格式和 lint 检查，零错误

### 通用

- 不使用裸 `new`/`delete`，统一用智能指针（`std::shared_ptr` / `std::make_shared`）
- 所有 public API 需要文档注释
- 错误处理：启动阶段 fail-fast（抛异常），运行时降级（记录日志后继续）
- 日志统一使用 `PatrolLogger`，禁止直接使用 `std::cout`、`RCLCPP_*`（除了 `RCLCPP_FATAL` 用于启动致命错误）

## 测试要求

- **测试框架**：GoogleTest (`ament_cmake_gtest`)
- **覆盖率目标**：每个模块的核心逻辑路径（正常路径 + 错误路径 + 边界值）
- **测试文件命名**：`test/<module_name>.cpp`，与源文件对应
- **运行命令**：`colcon test --packages-select patrol_bot` 全部通过
- 单元测试需覆盖：
  - ConfigLoader：完整解析、缺必填字段、可选字段默认值、类型错误、空数组
  - BatteryModel：放电消耗、充电恢复、上下界钳位、模式切换
  - PatrolLogger：文件创建、多级别写入、时间戳格式、flush
  - AlarmManager：消息完整性、warning vs critical
  - CameraBuffer：帧缓存、保存输出、无帧降级
  - Nav2ActionClient：mock Action Server、状态机
  - BT 节点：条件边界值、StatefulActionNode 生命周期、halt + 超时

### 集成测试

- 启动→巡逻→停止：状态转移、BT tick、Topic 数据
- 低电→充电→恢复：ReactiveFallback 抢占、断点保存/恢复
- Critical 异常→暂停：`patrol_state` 变 PAUSED
- 导航超时跳过：跳过当前点、日志、继续
- pause/resume：状态校验、BT 暂停/恢复

## 构建与验证命令（docker环境中）

```bash
# 编译
cd /root/patrol_robot_ws/
colcon build --packages-select patrol_bot_interfaces patrol_bot --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 运行测试
colcon test --packages-select patrol_bot

# C++ 静态分析
clang-tidy -p build/patrol_bot <source_files>
cppcheck --enable=all --suppress=missingIncludeSystem src/

# Python lint（仅 launch 脚本）
ruff check launch/
```

## 与已有代码的关系

- 项目已存在的 `nav2_demo` 包保持不动，与 `patrol_bot` 完全独立，互不依赖。
- `patrol_bot` 的所有代码从零开始编写，不复用 `nav2_demo` 的任何代码。
- 两者在同一工作空间（`patrol_robot_ws`）中共存。
