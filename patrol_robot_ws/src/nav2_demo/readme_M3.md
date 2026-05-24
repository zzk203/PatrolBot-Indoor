# M3：巡逻序列实现

> **目标**：加载预定义巡逻点，使用行为树逐个导航，实现基本巡逻循环。
> **技术栈**：ROS2 Humble + BehaviorTree.CPP v4 + Nav2 + yaml-cpp

---

## 模块文件清单

```
nav2_demo/
├── config/
│   └── waypoints.yaml                  ← M3.1 ★ 巡逻路径点配置（5 个巡逻点 + 充电点）
├── include/nav2_demo/
│   └── bt_nodes/
│       ├── waypoint_structs.hpp        ← M3.2  Waypoint / PatrolConfig 数据结构
│       ├── load_waypoints_node.hpp     ← M3.2 ★ 从 YAML 加载巡逻点的 BT 节点
│       ├── navigate_to_pose_node.hpp   ← M3.3 ★ 封装 navigate_to_pose 动作客户端
│       ├── next_waypoint_node.hpp      ← M3.4  顺序取出下一个巡逻点（索引自增）
│       └── record_failure_node.hpp     ← M3.5 ★ 记录堵赛/失败事件到黑板
├── src/bt_nodes/
│   ├── waypoint_structs.cpp            ← Waypoint → PoseStamped 转换
│   ├── load_waypoints_node.cpp         ← LoadWaypointsNode 实现 (yaml-cpp 解析)
│   ├── navigate_to_pose_node.cpp       ← NavigateToPoseNode 实现 (StatefulActionNode)
│   ├── next_waypoint_node.cpp          ← NextWaypointNode 实现
│   └── record_failure_node.cpp        ← RecordFailureNode 实现
├── behavior_trees/
│   ├── patrol_round.xml                ← M3.4 ★ 巡逻循环子树（遍历航点 + 停留）
│   └── patrol_main.xml                 ← M3.6 ★ 主巡逻行为树（集成入口）
├── tests/
│   ├── test_load_waypoints.cpp         ← M3.2  C++ GTest（parseYaml 单元测试）
│   ├── test_patrol_bt_nodes.cpp        ← M3.3-5 C++ GTest（BT 节点集成测试）
│   └── test_patrol_sequence.py         ← M3.6  Python 集成测试（Mock Action Server）
└── readme_M3.md                        ← 本文件
```

---

## 架构设计

### 行为树结构

```
patrol_main.xml
└── PatrolMain (Sequence)
    ├── Script: waypoints_file = "/path/to/waypoints.yaml"
    └── SubTree: PatrolRound
         │
         └── patrol_round.xml
              └── patrol_main_sequence (Sequence)
                   ├── 1. LoadWaypointsNode        ← 加载 YAML → 黑板
                   ├── 2. Script: current_index=0  ← 初始化索引
                   ├── 3. Script: failure_count=0  ← 初始化失败计数器
                   └── 4. Repeat (waypoints_count 次)
                        └── visit_single_waypoint (Sequence)
                             ├── NextWaypointNode   ← 取当前点 → PoseStamped
                             ├── Fallback (M3.5 堵赛放弃)
                             │    ├── NavigateToPoseNode ← 导航至目标
                             │    └── RecordFailureNode  ← 超时/失败记录
                             └── Wait                ← 停留 3 秒
```

### M3.5 堵赛放弃逻辑（Fallback 模式）

```
NavigateToPoseNode 超时 → 返回 FAILURE
         │
         ▼
Fallback 捕获 FAILURE → 执行 RecordFailureNode
         │
         ▼
RecordFailureNode:
  1. 递增 failure_count（黑板）
  2. 记录 last_failure_reason（黑板）
  3. 打印错误日志
  4. 返回 SUCCESS
         │
         ▼
Fallback 返回 SUCCESS → 巡逻继续执行下一个点
```

### 自定义 BT 节点一览

| 节点 | 基类 | 黑板输入 | 黑板输出 | 说明 |
|------|------|----------|----------|------|
| **LoadWaypointsNode** | SyncActionNode | `waypoints_file` | `patrol_waypoints`, `charge_standby`, `charge_dock`, `waypoints_count`, `navigation_timeout`, `waypoint_wait_duration` | YAML → 黑板，首次 tick 加载所有数据 |
| **NextWaypointNode** | SyncActionNode | `patrol_waypoints`, `current_index` | `current_waypoint` (PoseStamped) | 取点后索引自增，越界返回 FAILURE |
| **NavigateToPoseNode** | StatefulActionNode | `goal`, `timeout` | `nav_result` | 异步动作客户端，支持超时取消 |
| **RecordFailureNode** | SyncActionNode | `reason`, `failure_count` (双向) | `failure_count`, `last_failure_reason` | 记录失败后始终返回 SUCCESS |

---

## 各任务说明

### M3.1 巡逻点配置

`config/waypoints.yaml` 定义 5 个巡逻点（闭环路径）、充电预备点、充电桩坐标：

```yaml
patrol_points:
  - {x: 2.0, y: 2.0, yaw: 1.57}
  - {x: 4.0, y: 4.0, yaw: 3.14}
  - {x: 6.0, y: 2.0, yaw: -1.57}
  - {x: 4.0, y: 0.0, yaw: 0.0}
  - {x: 2.0, y: 2.0, yaw: 0.0}

charge_standby:  {x: 7.0, y: 6.0, yaw: -1.57}
charge_dock:     {x: 7.5, y: 6.0, yaw: -1.57}
navigation_timeout: 60.0
waypoint_wait_duration: 3.0
```

### M3.2 LoadWaypointsNode

- 使用 `yaml-cpp` 解析 YAML 文件
- 静态方法 `parseYaml()` 可单独测试（无 ROS 依赖）
- 向黑板写入 `patrol_waypoints` (vector\<Waypoint\>)、`waypoints_count` (int) 等
- 单元测试覆盖：标准解析、空列表、文件不存在、格式错误

### M3.3 NavigateToPoseNode

- 继承 `BT::StatefulActionNode`，支持异步操作
- `onStart()`: 创建/复用动作客户端，发送目标
- `onRunning()`: 检查结果到达 + 超时检测
- `onHalted()`: 取消当前导航目标
- 返回值：SUCCESS / FAILURE / RUNNING（超时时 FAILURE + `nav_result`="TIMEOUT"）

### M3.4 巡逻子树 (patrol_round.xml)

- `Repeat` 循环执行 `waypoints_count` 次
- 每次迭代：取点 → 导航 → 停留 3 秒
- `NextWaypointNode` 自动递增索引

### M3.5 堵赛放弃逻辑

- `Fallback{ NavigateToPoseNode | RecordFailureNode }`
- 超时/失败后记录事件到黑板（`failure_count`, `last_failure_reason`）
- 巡逻继续执行下一航点

---

## 运行测试

```bash
cd /home/zzk/PatrolBot-Indoor/patrol_robot_ws
colcon build --packages-select nav2_demo
source install/setup.bash

# M3 C++ GTest 测试
colcon test --packages-select nav2_demo \
  --ctest-args -R "test_load_waypoints|test_patrol_bt_nodes"

# M3 Python 集成测试
colcon test --packages-select nav2_demo \
  --ctest-args -R "test_patrol_sequence"

# M3 全部测试
colcon test --packages-select nav2_demo \
  --ctest-args -R "test_load_waypoints|test_patrol_bt_nodes|test_patrol_sequence"

# 查看测试结果
colcon test-result --all
```

---

## 测试覆盖

| 测试文件 | 模块 | 用例数 | 说明 |
|----------|------|--------|------|
| `test_load_waypoints.cpp` | M3.2 | 8 | parseYaml（标准/空/不存在/格式错误）+ Waypoint 转换 + BT 树集成 |
| `test_patrol_bt_nodes.cpp` | M3.3-5 | 9 | NextWaypointNode（3）+ RecordFailureNode（3）+ Fallback 协作（2）+ 集成 |
| `test_patrol_sequence.py` | M3.1-6 | 10 | 时序行为（3）+ 多航点（1）+ 失败恢复（2）+ 配置验证（4）+ XML 验证（3） |

---

## 使用指南

### 1. 启动完整仿真 + 巡逻

```bash
# 构建
cd /home/zzk/PatrolBot-Indoor/patrol_robot_ws
colcon build --packages-select nav2_demo
source install/setup.bash

# 启动仿真 + Nav2 栈
ros2 launch nav2_demo patrol_sim.launch.py
```

### 2. 运行巡逻序列

```bash
# 方法一：通过行为树导航器（需要将 patrol_main.xml 设为 default_bt_xml_filename）
# 修改 nav2_params.yaml 中的 default_bt_xml_filename 为 patrol_main.xml

# 方法二：通过独立的巡逻节点（待后续实现）
# ros2 run nav2_demo patrol_node
```

### 3. 观察行为

- 机器人按顺序访问 5 个巡逻点
- 每个点停留 3 秒
- 导航超时（60 秒）后自动跳过当前点
- 失败次数记录在黑板中

---

## 模块依赖

| 依赖包 | 用途 |
|--------|------|
| `behaviortree_cpp` | BehaviorTree.CPP v4 核心库 |
| `libyaml-cpp-dev` | YAML 文件解析 |
| `rclcpp_action` | ROS2 动作客户端 |
| `nav2_msgs` | NavigateToPose action 定义 |
