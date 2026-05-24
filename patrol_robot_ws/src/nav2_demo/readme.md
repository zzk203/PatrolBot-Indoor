# PatrolBot 室内巡逻机器人 — 使用指南

> nav2_demo 包是 PatrolBot 项目的核心 ROS2 包，集成了 Gazebo 仿真、Nav2 导航、行为树任务编排、电池回充及异常处理等全部功能。

---

## 前置依赖

- **ROS2 Humble** + Gazebo Fortress (Ignition)
- **ros_gz** 桥接包
- **Navigation2**: `ros-humble-navigation2` `ros-humble-nav2-bringup`
- **BehaviorTree.CPP v4**: `ros-humble-behaviortree-cpp`
- **yaml-cpp**: `libyaml-cpp-dev`
- **sensor_msgs** / **nav_msgs** / **geometry_msgs**

在 Docker 环境中以上已全部预装，宿主环境需自行安装。

---

## 文件结构

```
nav2_demo/
├── worlds/patrol_world.sdf              # 10m×10m 室内仿真世界（墙壁/障碍/地面/光源）
├── models/patrol_bot/
│   ├── model.sdf                        # 机器人 SDF（差分驱动/LiDAR/相机）
│   └── model.urdf                       # TF 树定义（base_footprint→base_link→lidar→camera）
├── maps/office.pgm + office.yaml        # 占据栅格地图（200×200 @ 0.05m）
├── config/
│   ├── nav2_params.yaml                 # Nav2 全局参数（规划/控制/AMCL/代价地图）
│   ├── waypoints.yaml                   # 巡逻点/充电桩/超时配置
│   └── gz_bridge.yaml                   # Gazebo 桥接备用配置
├── behavior_trees/
│   ├── patrol_main.xml                  # ★ 主行为树入口（轮次调度+低电中断）
│   ├── patrol_round.xml                 # 巡逻循环子树
│   ├── charge_recovery.xml              # 回充对接子树
│   └── navigate_w_replanning_and_recovery.xml  # Nav2 内置行为树
├── launch/patrol_sim.launch.py          # ★ 一键启动全部节点
├── rviz/nav2_view.rviz                  # RViz2 可视化配置
├── scripts/
│   ├── keyboard_teleop.py               # 键盘遥控
│   ├── odom_republisher.py              # 里程计帧名转换
│   ├── pointcloud_to_scan.py            # 点云→LaserScan
│   ├── static_map_publisher.py          # 静态地图发布
│   ├── nav2_single_goal_test.py         # 单点导航测试脚本
│   └── dynamic_obstacle_test.py         # 动态避障测试脚本
├── src/
│   ├── nav2_demo_node.cpp               # NavigateToPose 动作客户端
│   ├── battery_simulator_node.cpp       # 电池充放电模拟
│   └── bt_nodes/                        # 14 个自定义行为树节点
├── include/nav2_demo/                   # C++ 头文件
├── tests/                               # 15 个测试文件（C++ GTest + Python pytest）
└── readme_M1~M7.md                      # 各模块详细文档
```

---

## 构建

```bash
cd ~/patrol_robot_ws
colcon build --packages-select nav2_demo
source install/setup.bash
```

---

## 运行

### 方式一：一键启动完整仿真（推荐）

```bash
source install/setup.bash
ros2 launch nav2_demo patrol_sim.launch.py
```

此命令会启动：Gazebo 仿真世界 → ros_gz_bridge 桥接 → 里程计/TF → 点云转换 → 地图服务 → AMCL → Nav2 导航栈 → 电池模拟 → RViz2。

Gazebo 和 RViz2 窗口会自动打开。在 RViz2 中点击 **"2D Goal Pose"** 按钮，在地图上点击目标点即可指挥机器人导航。

### 方式二：启用键盘遥控

```bash
ros2 launch nav2_demo patrol_sim.launch.py enable_keyboard:=true
```

| 按键 | 功能 |
|------|------|
| `w` / `s` | 前进 / 后退 |
| `a` / `d` | 左转 / 右转 |
| 空格 | 急停 |
| `q` | 退出 |

### 方式三：命令行单点导航测试

在仿真运行后，另开终端：

```bash
source install/setup.bash
ros2 run nav2_demo nav2_single_goal_test.py --ros-args -p x:=4.0 -p y:=4.0 -p yaw:=0.0
```

### 方式四：运行完整巡逻任务（行为树驱动）

一键启动后，巡逻任务由行为树自动执行，也可以手动发布导航目标。行为树会在后台运行低电检测和回充逻辑。

---

## 行为树架构

```
patrol_main.xml (顶层)
└─ Repeat(轮次循环)
   └─ Sequence(单轮)
      ├─ Repeat(∞)  ← 巡逻内层循环（低电可中断）
      │  └─ Inverter(巡逻完成条件) → 跳出
      │  └─ Fallback
      │     ├─ ReactiveSequence{BatteryMonitor | PatrolRound}
      │     └─ Sequence{IsBatteryLow | ChargeRecovery}  ← 低电回充
      ├─ Fallback{NavigateToDockPrep | AlwaysSuccess}   ← 返回待命点
      ├─ Wait(轮次间隔)                                   ← 等待下一轮
      └─ Script(重置索引)                                 ← 轮次清零
```

**数据流**: `waypoints.yaml` → `LoadWaypointsNode` → 黑板(waypoints) → `NextWaypointNode` → `NavigateToPoseNode` → Nav2 动作客户端

**低电流程**: `BatteryMonitorNode` 检测电量 < 20% → ReactiveSequence 中断巡逻 → `ChargeRecovery`: 导航到充电预备点 → 视觉伺服对准充电桩 → 10s 模拟充电 → 满电恢复巡逻（断点续巡）

---

## 关键话题与 TF 树

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/model/patrol_bot/cmd_vel` | Twist | ROS→GZ | 速度指令 |
| `/model/patrol_bot/odometry` | Odometry | GZ→ROS | 原始里程计 |
| `/odom` | Odometry | — | 帧名转换后里程计 |
| `/scan` | LaserScan | — | 2D 激光扫描 |
| `/camera/image_raw` | Image | GZ→ROS | RGB 相机图像 |
| `/map` | OccupancyGrid | — | 静态地图 |
| `/battery_state` | BatteryState | — | 电池状态 |
| `/patrol_alerts` | String | — | 异常报警 |

**TF 树**: `map → odom → base_footprint → base_link → {chassis, lidar_link, camera_link}`

---

## 参数调优

### 巡逻配置 (waypoints.yaml)

```yaml
patrol_points:      # 巡逻路径点 (至少3个，构成闭环)
  - {x: 2.0, y: 2.0, yaw: 1.57}
  # ...
charge_standby:     # 充电预备点（dock 前方 0.5m）
  x: 7.0, y: 6.0, yaw: -1.57
charge_dock:        # 充电桩最终停靠位置
  x: 7.5, y: 6.0, yaw: -1.57
navigation_timeout: 60.0       # 导航超时（秒）
waypoint_wait_duration: 3.0    # 到达巡逻点后停留时间
```

### 电池参数 (launch 中配置)

```python
{
    "initial_percentage": 100.0,  # 初始电量 %
    "discharge_rate": 0.5,        # 放电速率 %/s
    "charge_rate": 5.0,           # 充电速率 %/s
    "publish_rate": 1.0,          # 发布频率 Hz
}
```

### 视觉伺服参数 (VisualServoNode)

- 对准成功条件：角度偏差 < 0.05 rad，横向偏差 < 0.02 m
- P 控制器增益：`linear_gain`, `angular_gain`
- 超时时间：15 秒
- 重试次数：3 次（RetryNode）

---

## 运行测试

```bash
# 构建并运行全部测试
colcon build --packages-select nav2_demo && colcon test --packages-select nav2_demo

# 查看测试结果
colcon test-result --all

# 运行特定模块测试
colcon test --packages-select nav2_demo --ctest-args -R "test_battery"       # M4
colcon test --packages-select nav2_demo --ctest-args -R "test_m5_docking"    # M5
colcon test --packages-select nav2_demo --ctest-args -R "test_m7"            # M7
```

### 测试覆盖

| 测试文件 | 类型 | 用例数 | 覆盖模块 |
|----------|------|--------|----------|
| test_sim_launch.py | pytest | 22 | M1 全部 |
| test_keyboard_teleop.py | pytest | 10 | M1.3 |
| test_odom_republisher.py | pytest | 8 | M1.4 |
| test_navigate_to_pose.py | pytest | 11 | M2.3 |
| test_nav2_demo_node.py | pytest | 10 | M2.4 |
| test_dynamic_obstacle.py | pytest | 11 | M2.5 |
| test_load_waypoints.cpp | GTest | 8 | M3.2 |
| test_patrol_bt_nodes.cpp | GTest | 9 | M3.3-5 |
| test_patrol_sequence.py | pytest | 10 | M3.6 |
| test_battery_simulator.cpp | GTest | 8 | M4.1 |
| test_battery_bt_nodes.cpp | GTest | 11 | M4.2-5 |
| test_m5_docking.cpp | GTest | 14 | M5 全部 |
| test_m6_round_scheduling.cpp | GTest | 9 | M6 全部 |
| test_m7_exceptions.cpp | GTest | 14 | M7 全部 |

---

## 常见问题

**Q: Gazebo 窗口没有出现？**  
A: Docker 环境下需要 X11 转发，确保运行 `xhost +` 后再启动容器。

**Q: 机器人不移动？**  
A: 检查 `/model/patrol_bot/cmd_vel` 话题是否有数据：`ros2 topic echo /model/patrol_bot/cmd_vel`。确认 `odom_republisher` 和 `ros_gz_bridge` 节点正常运行。

**Q: 导航规划失败？**  
A: 确认 AMCL 已收敛（RViz2 中粒子云收拢），在 RViz2 中用 "2D Pose Estimate" 工具手动指定初始位姿。

**Q: 如何修改巡逻路线？**  
A: 编辑 `config/waypoints.yaml` 中的 `patrol_points` 列表，然后重启仿真。

**Q: 低电回充不触发？**  
A: 放电速率默认 0.5%/s，需 160 秒才会降到 20%。可在 launch 中调大 `discharge_rate` 以加速测试。
