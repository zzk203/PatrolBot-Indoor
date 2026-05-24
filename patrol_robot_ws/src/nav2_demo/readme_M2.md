# M2：Nav2 导航集成

> **目标**：加载静态地图，配置 AMCL 定位，实现单点自主导航。
> **技术栈**：ROS2 Humble + Nav2 (Navigation2) + BehaviorTree.CPP v4

---

## 模块文件清单

```
nav2_demo/
├── maps/
│   ├── office.pgm                    ← M2.1 占据栅格地图（10m×10m）
│   └── office.yaml                   ← M2.1 地图元数据
├── config/
│   └── nav2_params.yaml              ← M2.2 ★ Nav2 完整参数配置
├── launch/
│   └── patrol_sim.launch.py          ← M2.2 ★ 一键启动（含 Nav2 完整栈）
├── include/nav2_demo/
│   └── nav2_demo_node.hpp            ← M2.4 封装的动作客户端头文件
├── src/
│   └── nav2_demo_node.cpp            ← M2.4 重构的导航节点实现
├── tests/
│   ├── test_navigate_to_pose.py      ← M2.3 单点导航 Mock 测试
│   ├── test_nav2_demo_node.py        ← M2.4 动作客户端 Mock 测试
│   └── test_dynamic_obstacle.py      ← M2.5 动态避障参数验证
├── scripts/
│   ├── nav2_single_goal_test.py      ← M2.3 命令行单点导航测试
│   └── dynamic_obstacle_test.py      ← M2.5 动态障碍物避障测试
├── behavior_trees/
│   └── navigate_w_replanning_and_recovery.xml  ← Nav2 行为树
└── rviz/
    └── nav2_view.rviz                ← RViz2 可视化配置
```

---

## Nav2 架构概览

```
                    ┌──────────────────────────────────────────────┐
                    │              Nav2 导航栈                      │
                    │                                               │
                    │  ┌──────────┐   ┌──────────┐                │
                    │  │ map_server│   │   AMCL   │ ← 粒子滤波定位  │
                    │  │ (静态地图) │   │ (定位)    │                │
                    │  └──────────┘   └─────┬────┘                │
                    │                       │ map→odom TF          │
                    │  ┌────────────────────┴──────────────┐       │
                    │  │        bt_navigator               │       │
                    │  │  (行为树调度: 规划→执行→恢复)       │       │
                    │  └──────────┬───────────────────────┘        │
                    │             │                                 │
                    │  ┌──────────▼──────────┐   ┌──────────────┐  │
                    │  │   planner_server    │   │behavior_server│  │
                    │  │  (全局规划: A*/NavFN)│   │(Spin/BackUp)  │  │
                    │  └──────────┬──────────┘   └──────────────┘  │
                    │             │ /plan                          │
                    │  ┌──────────▼──────────┐                     │
                    │  │  controller_server  │                     │
                    │  │ (局部控制: Regulated │                     │
                    │  │  Pure Pursuit)      │                     │
                    │  └──────────┬──────────┘                     │
                    │             │ cmd_vel                        │
                    └─────────────┼────────────────────────────────┘
                                  │
                    ┌─────────────▼──────────────┐
                    │   Gazebo DiffDrive 插件     │
                    │   → 机器人运动              │
                    └────────────────────────────┘
```

### 关键数据流

| 数据 | 发布者 | 话题/服务 | 消费者 |
|------|--------|-----------|--------|
| 静态地图 | `static_map_publisher` | `/map` | global_costmap, RViz2 |
| 定位 TF | AMCL | `map→odom` | costmaps, bt_navigator |
| 全局路径 | planner_server | `/plan` | controller_server, RViz2 |
| 速度指令 | controller_server | `/model/patrol_bot/cmd_vel` | Gazebo DiffDrive |
| 导航动作 | nav2_demo_node | `navigate_to_pose` (Action) | bt_navigator |

---

## 各任务说明

### M2.1 静态地图

地图已预生成（可用 SLAM Toolbox 或 Cartographer 在实际环境中重建）：

```yaml
# maps/office.yaml
image: office.pgm
mode: trinary
resolution: 0.05          # 5cm/px
origin: [-5.0, -5.0, 0.0] # 地图原点
occupied_thresh: 0.65
free_thresh: 0.25
```

- 尺寸：200×200 px → 10m×10m
- 格式：PGM (P5, 灰度)
- 与 `patrol_world.sdf` 的墙壁/障碍物布局匹配

### M2.2 Nav2 参数配置

`config/nav2_params.yaml` 包含完整的 Nav2 配置：

| 配置块 | 关键参数 | 说明 |
|--------|----------|------|
| **planner_server** | NavfnPlanner, tolerance=2.0 | 全局路径规划 |
| **controller_server** | RegulatedPurePursuit, 20Hz | 局部路径跟踪 |
| **bt_navigator** | navigate_w_replanning_and_recovery.xml | 行为树导航 |
| **behavior_server** | Spin/BackUp/DriveOnHeading/Wait | 恢复行为 |
| **global_costmap** | static_layer + inflation_layer | 全局代价地图 |
| **local_costmap** | inflation_layer, rolling_window | 局部代价地图 |
| **amcl** | DifferentialMotionModel, 500-2000 particles | 自适应定位 |
| **waypoint_follower** | WaitAtWaypoint | 航点跟随 |

关键参数调整说明：
- **RegulatedPurePursuit** 启用了 `use_regulated_linear_velocity_scaling`（动态避障自动减速）
- **局部代价地图** 启用了 `rolling_window`（跟随机器人移动，适应动态环境）
- **膨胀半径** 0.55m（机器人半径 0.22m），为避障留出空间

### M2.3 单点导航测试

测试方法：
1. 启动完整仿真
2. 使用 RViz2 的 "2D Goal Pose" 工具在地图上点击目标
3. 观察机器人路径规划和运动执行

自动化测试（Mock Action Server 验证）：
```
colcon test --packages-select nav2_demo --ctest-args -R test_navigate_to_pose
```

命令行测试脚本：
```bash
# 在完整仿真运行中执行
ros2 run nav2_demo nav2_single_goal_test.py \
  --ros-args -p x:=2.0 -p y:=1.5 -p yaw:=1.57 -p timeout:=30.0
```

### M2.4 navigate_to_pose 动作客户端

封装的动作客户端 (`nav2_demo_node`) 提供：

| 功能 | 方法 | 说明 |
|------|------|------|
| 发送航点 | `send_next_goal()` | 顺序发送预配置的航点 |
| 取消目标 | `cancel_current_goal()` | 取消当前正在执行的导航 |
| 反馈回调 | `feedback_callback()` | 显示剩余距离和预计时间 |
| 结果回调 | `result_callback()` | 处理 SUCCEEDED/CANCELED/ABORTED |
| 参数化航点 | `load_goals_from_parameters()` | 从 ROS2 参数加载航点列表 |

参数配置示例：
```bash
ros2 run nav2_demo nav2_demo_node \
  --ros-args -p goals.x:=[1.0,2.0,3.0] \
  -p goals.y:=[0.0,1.0,2.0] \
  -p goals.yaw:=[0.0,1.57,3.14]
```

### M2.5 动态避障

Nav2 的动态避障能力依赖于：

1. **局部代价地图滚动窗口** — 实时更新机器人周围障碍物
2. **Regulated Pure Pursuit 速度缩放** — 接近障碍物时自动减速
3. **恢复行为链** — Spin → BackUp → Wait，逐级尝试脱困
4. **行为树重规划** — 路径堵塞时触发全局重规划

验证脚本：
```bash
# 参数验证（无需仿真）
colcon test --packages-select nav2_demo --ctest-args -R test_dynamic_obstacle

# 运行时测试（需要仿真运行中）
ros2 run nav2_demo dynamic_obstacle_test.py \
  --ros-args -p goal_x:=3.0 -p goal_y:=2.0 \
  -p obstacle_x:=1.5 -p obstacle_y:=1.0
```

---

## 使用指南

### 1. 完整启动（仿真 + Nav2 + RViz2）

```bash
cd /home/zzk/PatrolBot-Indoor/patrol_robot_ws
colcon build --packages-select nav2_demo
source install/setup.bash

# 一键启动全部
ros2 launch nav2_demo patrol_sim.launch.py
```

### 2. 初始化定位（RViz2 中操作）

1. 点击 RViz2 工具栏的 **"2D Pose Estimate"**
2. 在地图上机器人起始位置单击并拖拽方向
3. 观察 AMCL 粒子云收敛

### 3. 发送导航目标

**方法 A：RViz2**
1. 点击 **"2D Goal Pose"** 工具
2. 在目标位置单击并拖拽方向
3. 观察机器人自动规划路径并移动

**方法 B：命令行**
```bash
# 启动 nav2_demo_node（会自动发送预配置航点）
ros2 run nav2_demo nav2_demo_node

# 或发送单点导航测试目标
ros2 run nav2_demo nav2_single_goal_test.py \
  --ros-args -p x:=2.0 -p y:=1.5 -p yaw:=1.57
```

### 4. 验证导航状态

```bash
# 查看导航反馈
ros2 topic echo /navigate_to_pose/_action/feedback

# 查看全局路径
ros2 topic echo /plan

# 查看代价地图
ros2 topic echo /local_costmap/costmap_raw

# 查看 AMCL 粒子
ros2 topic echo /particlecloud

# 查看 TF 树
ros2 run tf2_tools view_frames.py
```

---

## 运行测试

```bash
# 构建
cd /home/zzk/PatrolBot-Indoor/patrol_robot_ws
colcon build --packages-select nav2_demo
source install/setup.bash

# M2 全部测试
colcon test --packages-select nav2_demo \
  --ctest-args -R "test_navigate_to_pose|test_nav2_demo_node|test_dynamic_obstacle"

# 查看结果
colcon test-result --all
```

### 测试覆盖

| 测试文件 | 模块 | 用例数 | 说明 |
|----------|------|--------|------|
| `test_navigate_to_pose.py` | M2.3 | 11 | 单点导航（目标格式、坐标范围、四元数、反馈、结果状态） |
| `test_nav2_demo_node.py` | M2.4 | 10 | 动作客户端（创建/发送/反馈/结果/取消/拒绝/多航点/超时） |
| `test_dynamic_obstacle.py` | M2.5 | 11 | 动态避障参数（控制器、代价地图、恢复行为、行为树、综合评估） |

---

## 常见问题

### Q: 导航目标被拒绝？
- 确保 AMCL 定位已初始化（在 RViz2 中使用 2D Pose Estimate）
- 检查目标点是否在地图可行区域

### Q: 机器人不移动？
- 检查 `cmd_vel` 话题是否正确映射到 `/model/patrol_bot/cmd_vel`
- 检查局部规划器是否收到 `/plan`
- 运行 `ros2 topic echo /model/patrol_bot/cmd_vel` 查看速度指令

### Q: 定位漂移/丢失？
- 增加 AMCL `min_particles` 参数
- 检查激光雷达数据是否正常（`ros2 topic echo /scan`）
- 在 RViz2 中重新初始化定位

### Q: 动态避障效果不佳？
- 减小 `inflation_radius` 使路径更紧凑
- 增大 `controller_frequency` 提高响应速度
- 检查 `use_regulated_linear_velocity_scaling` 是否启用

---

## 模块依赖

| 依赖包 | 用途 |
|--------|------|
| `nav2_amcl` | 自适应蒙特卡洛定位 |
| `nav2_planner` | 全局路径规划（Navfn/RRT*） |
| `nav2_controller` | 局部路径跟踪（Regulated Pure Pursuit） |
| `nav2_behaviors` | 旋转/后退等恢复行为 |
| `nav2_bt_navigator` | 行为树导航调度 |
| `nav2_lifecycle_manager` | 生命周期节点管理 |
| `nav2_costmap_2d` | 全局/局部代价地图 |
| `nav2_msgs` | Nav2 消息/动作定义 |
| `nav2_navfn_planner` | Navfn 全局规划器 |
| `nav2_regulated_pure_pursuit_controller` | 调节型 Pure Pursuit 控制器 |
