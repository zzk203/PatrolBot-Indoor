# M1：仿真环境与基础移动

> **目标**：搭建室内仿真世界，导入差速机器人模型，实现键盘控制和基础里程计显示。
> **技术栈**：ROS2 Humble + Gazebo Fortress + Nav2

---

## 模块文件清单

```
nav2_demo/
├── worlds/
│   └── patrol_world.sdf              ← M1.1 仿真世界（10m×10m 室内 + 障碍物）
├── models/patrol_bot/
│   ├── model.sdf                     ← M1.2 机器人 SDF（DiffDrive + LiDAR + Camera）
│   └── model.urdf                    ← M1.2 URDF 定义 TF 树
├── scripts/
│   ├── keyboard_teleop.py            ← M1.3 键盘遥控（w/s/a/d + 空格急停）
│   ├── odom_republisher.py           ← M1.4 里程计重发布 + TF 广播
│   ├── pointcloud_to_scan.py         ← 点云 → LaserScan 转换
│   ├── static_map_publisher.py       ← 静态地图发布（PGM → /map）
│   └── gazebo_diff_drive.sh          ← 差速小车独立演示脚本
├── launch/
│   └── patrol_sim.launch.py          ← ★ 一键启动（Gazebo + 桥接 + Nav2 + RViz2）
├── tests/
│   ├── test_keyboard_teleop.py       ← M1.3 键盘遥控单元测试
│   ├── test_odom_republisher.py      ← M1.4 里程计重发布单元测试
│   └── test_sim_launch.py            ← M1.5 启动/配置/RViz 验证测试
├── config/
│   ├── gz_bridge.yaml                ← ROS2 ↔ Gazebo 话题桥接配置
│   └── nav2_params.yaml              ← Nav2 导航参数
├── maps/
│   ├── office.pgm                    ← 占据栅格地图（10m×10m）
│   └── office.yaml                   ← 地图元数据
├── rviz/
│   └── nav2_view.rviz                ← RViz2 可视化配置
└── CMakeLists.txt                    ← 构建配置
```

---

## 数据流全景

```text
                    Gazebo 仿真世界 (patrol_world.sdf)
                    ┌───────────────────────────────┐
                    │  patrol_bot                    │
                    │  ├─ DiffDrive Plugin           │
                    │  │  /cmd_vel  ← 速度指令       │
                    │  │  /odometry → 里程计         │
                    │  ├─ GPU LiDAR → /scan (点云)   │
                    │  └─ Camera   → /camera/image   │
                    └──────────┬────────────────────┘
                               │
                          ros_gz_bridge
                     (话题双向翻译/桥接)
                               │
          ┌────────────────────┼────────────────────┐
          ▼                    ▼                    ▼
    /model/patrol_bot/    /model/patrol_bot/    /scan (PointCloud2)
    cmd_vel (Twist)       odometry (Odometry)    │
          │                    │                  │
    keyboard_teleop       odom_republisher   pointcloud_to_scan
    (键盘 → Twist)        (帧名转换+TF)      (点云 → LaserScan)
          │                    │                  │
          └────┐               ▼                  ▼
               │          /odom + TF           /scan (LaserScan)
               │          (odom→base_footprint)
               └──────────────┼──────────────────┘
                              ▼
                     Nav2 导航栈 + RViz2
```

---

## 使用指南

### 1. 完整启动（仿真 + 导航 + RViz2）

```bash
# 构建
cd /home/zzk/PatrolBot-Indoor/patrol_robot_ws
colcon build --packages-select nav2_demo
source install/setup.bash

# 启动全部（Gazebo 世界 + 桥接 + 里程计 + 点云转换 + 地图 + Nav2 + RViz2）
ros2 launch nav2_demo patrol_sim.launch.py
```

### 2. 启动键盘遥控（配合完整启动）

在另一个终端（Docker 内）：
```bash
source install/setup.bash
ros2 launch nav2_demo patrol_sim.launch.py enable_keyboard:=true
```

或单独运行：
```bash
source install/setup.bash
ros2 run nav2_demo keyboard_teleop.py
```

**按键映射**：
| 按键 | 动作     | 说明               |
|------|----------|--------------------|
| `w`  | 前进     | 线速度 +0.3 m/s    |
| `s`  | 后退     | 线速度 -0.3 m/s    |
| `a`  | 左转     | 角速度 +1.0 rad/s  |
| `d`  | 右转     | 角速度 -1.0 rad/s  |
| 空格 | 急停     | 所有速度归零       |
| `q`  | 退出     | 停止节点           |

参数调节：
```bash
ros2 run nav2_demo keyboard_teleop.py \
  --ros-args -p linear_vel:=0.5 -p angular_vel:=2.0
```

### 3. 独立 Gazebo 差速小车演示

```bash
bash scripts/gazebo_diff_drive.sh
```
自动演示前进 → 左转 → 右转 → 弧线 → 后退 → 停止。

### 4. 手工验证各话题

```bash
# 查看里程计
ros2 topic echo /odom

# 查看激光扫描
ros2 topic echo /scan

# 查看 TF 树
ros2 run tf2_tools view_frames.py

# 查看所有活跃话题
ros2 topic list
```

---

## 架构说明

### M1.1 仿真世界 (`patrol_world.sdf`)
- **环境**：10m × 10m 封闭室内空间
- **墙壁**：四面墙体（北/南/东/西）
- **障碍物**：1 根圆柱 + 4 个不同形状的静态障碍物
- **地面**：灰调平面，便于 LiDAR 反射
- **光照**：方向光，带阴影
- **机器人**：patrol_bot 内联模型（DiffDrive + LiDAR + Camera）

### M1.2 差速机器人模型
- **底盘**：0.4m × 0.3m × 0.15m（蓝色）
- **驱动轮**：左右各一（半径 0.05m，间距 0.36m）
- **万向轮**：后部支撑
- **LiDAR**：GPU LiDAR，360° 扫描，12m 量程
- **相机**：RGB 虚拟相机（640×480，为 M5 视觉模块预留）
- **TF 树**：`odom → base_footprint → base_link → lidar_link → camera_link`

### M1.3 键盘控制
- 节点 `keyboard_teleop.py`，话题 `/model/patrol_bot/cmd_vel`
- 通过 `ros_gz_bridge` 桥接到 Gazebo DiffDrive 插件
- 通过 `enable_keyboard` 参数控制是否随 launch 启动
- 速度参数可调（linear_vel / angular_vel）

### M1.4 里程计和 TF
- Gazebo DiffDrive 输出原生话题 `/model/patrol_bot/odometry`
- `odom_republisher` 负责：
  1. 帧名转换：`patrol_bot/odom` → `odom`，`patrol_bot/chassis` → `base_footprint`
  2. 广播 TF：`odom` → `base_footprint` 动态变换
- `robot_state_publisher` 发布静态 TF：`base_footprint` → `base_link` → `lidar_link`

### M1.5 RViz2 可视化
- 配置 `nav2_view.rviz` 包含：
  - `/map` 占据栅格地图
  - `/global_costmap/costmap` 全局代价地图
  - `/local_costmap/costmap` 局部代价地图
  - `/plan` 全局路径
  - RobotModel（机器人模型跟随 TF）
  - `/scan` 激光扫描可视化
  - `/particlecloud` AMCL 定位粒子
  - `/goal_pose_marker` 导航目标标记
- 固定坐标系：`map`
- 2D Goal Pose 工具：设定导航目标

---

## 运行测试

```bash
# 构建并运行所有 M1 测试
cd /home/zzk/PatrolBot-Indoor/patrol_robot_ws
colcon build --packages-select nav2_demo
colcon test --packages-select nav2_demo --ctest-args -R "test_keyboard_teleop|test_odom_republisher|test_sim_launch"
colcon test-result --all

# 或单独运行
colcon test --packages-select nav2_demo --ctest-args -R test_keyboard_teleop
colcon test --packages-select nav2_demo --ctest-args -R test_odom_republisher
colcon test --packages-select nav2_demo --ctest-args -R test_sim_launch
```

### 测试内容

| 测试文件                    | 覆盖模块 | 测试项数 | 说明                           |
|-----------------------------|----------|----------|--------------------------------|
| `test_keyboard_teleop.py`   | M1.3     | 10       | 参数、键值映射、边界情况       |
| `test_odom_republisher.py`  | M1.4     | 8        | 初始化、帧名转换、TF 广播      |
| `test_sim_launch.py`        | M1.1-5   | 22       | 文件存在性、配置完整性、语法   |

---

## 常见问题

### Q: Gazebo 启动后看不到机器人？
- 确保构建后 `source install/setup.bash`
- 检查 `ros_gz_bridge` 是否正确桥接
- 运行 `ros2 topic list` 确认话题存在

### Q: 键盘遥控无法控制？
- 确保桥接话题匹配：`/model/patrol_bot/cmd_vel`
- 检查 `ros_gz_bridge` 输出日志是否有错误
- 先运行 `ign topic -l` 确认 Gazebo 端的话题

### Q: 里程计 /odom 话题为空？
- 确认 `odom_republisher` 正在运行
- 检查输入话题 `/model/patrol_bot/odometry` 是否有数据
- 用 `ros2 topic echo /model/patrol_bot/odometry` 验证

### Q: RViz2 不显示激光数据？
- 确保 `/scan` 话题有数据（`ros2 topic echo /scan`）
- 检查点云转换节点是否运行
- 在 RViz2 中检查 LaserScan 显示配置

---

## 模块依赖

| 依赖包                | 用途                     |
|-----------------------|--------------------------|
| `ros_gz_bridge`       | ROS2 ↔ Gazebo 话题桥接   |
| `ros_gz_sim`          | Gazebo Fortress ROS2 接口|
| `robot_state_publisher` | URDF 加载 + TF 广播    |
| `rviz2`               | 可视化                   |
| `nav2_amcl`           | 自适应蒙特卡洛定位       |
| `nav2_planner`        | 全局路径规划             |
| `nav2_controller`     | 局部路径跟踪             |
| `nav2_behaviors`      | 旋转/后退等恢复行为      |
| `nav2_bt_navigator`   | 行为树导航器             |
| `nav2_lifecycle_manager` | 生命周期节点管理      |
| `sensor_msgs_py`      | PointCloud2 Python 工具  |
| `tf2_ros`             | TF 广播/监听             |
