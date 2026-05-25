# PatrolBot-Indoor

室内巡逻机器人系统，基于 ROS2 Humble + BehaviorTree.CPP v4 + Nav2 + Gazebo Fortress。

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

## 功能

- 按 YAML 配置的固定路线自动巡逻
- 低电自动导航回充电桩，充电后断点恢复
- 异常检测报警（warning/critical 两级）
- 巡逻点拍照存档
- ROS2 Service 启停控制
- 日志实时输出到控制台和文件

## 目录结构

```
patrol_robot_ws/src/
├── patrol_bot_interfaces/      # 自定义 ROS2 消息包
│   └── msg/
│       ├── PatrolStatus.msg
│       └── PatrolAlarm.msg
└── patrol_bot/                  # 主业务包
    ├── include/patrol_bot/      # 头文件
    ├── src/                     # 源文件
    │   └── bt_nodes/            # 行为树节点
    │       ├── conditions/      # 条件节点
    │       └── actions/         # 动作节点
    ├── bt_xml/                  # 行为树 XML
    ├── config/                  # 配置文件
    ├── launch/                  # 启动文件
    ├── scripts/                 # 控制脚本
    └── test/                    # 单元测试
```

## 构建

```bash
# 配置/进入ros2镜像
bash docker/dev.sh

cd /root/patrol_robot_ws
source /opt/ros/humble/setup.bash

# 编译
colcon build --packages-select patrol_bot_interfaces patrol_bot

# 测试
colcon test --packages-select patrol_bot --return-code-on-test-failure

# 静态分析
cppcheck --enable=all --suppress=missingIncludeSystem src/patrol_bot/src/ src/patrol_bot/include/
find src/patrol_bot -name '*.cpp' | xargs clang-tidy -p build/patrol_bot
```

## 运行

```bash
source install/setup.bash

# 仅启动巡逻节点
ros2 launch patrol_bot patrol_bot.launch.py

# 完整仿真（Gazebo + Nav2 + Patrol）
ros2 launch patrol_bot patrol_simulation.launch.py

# 控制命令
ros2 service call /patrol/start_patrol std_srvs/srv/Trigger "{}"
ros2 service call /patrol/pause_patrol std_srvs/srv/Trigger "{}"
ros2 service call /patrol/resume_patrol std_srvs/srv/Trigger "{}"
ros2 service call /patrol/stop_patrol std_srvs/srv/Trigger "{}"

# 查看状态
ros2 topic echo /patrol/status
```

## 配置文件

编辑 `config/patrol_config.yaml` 可调整巡逻路线、充电桩位置、电池参数等，无需修改代码。

## 设计文档

- `doc/proposal.md` — 需求文档
- `doc/detailed_design.md` — 详细设计
- `doc/tasks/` — 各模块子任务清单
