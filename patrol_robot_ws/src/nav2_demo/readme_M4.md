# M4：电池模拟与低电回充

> **目标**：模拟电池状态，实现低电自动中断巡逻并返回充电桩，充电完成后恢复巡逻。
> **技术栈**：ROS2 Humble + BehaviorTree.CPP v4 + sensor_msgs/BatteryState

---

## 模块文件清单

```
nav2_demo/
├── include/nav2_demo/
│   ├── battery_simulator_node.hpp           ← M4.1 ★ 电池模拟节点
│   └── bt_nodes/
│       ├── battery_monitor_node.hpp         ← M4.2 ★ 电池状态监测条件节点
│       ├── is_battery_low_condition.hpp     ← M4.3   低电判断条件节点
│       └── set_charging_mode_node.hpp       ← M4.4 ★ 设置充电模式动作节点
├── src/
│   ├── battery_simulator_node.cpp           ← M4.1 ★ 电池模拟节点实现
│   └── bt_nodes/
│       ├── battery_monitor_node.cpp         ← M4.2 ★ 电池状态监测 BT 节点
│       ├── is_battery_low_condition.cpp     ← M4.3   低电判断 BT 节点
│       └── set_charging_mode_node.cpp       ← M4.4 ★ 充电模式 BT 节点
├── behavior_trees/
│   ├── patrol_main.xml                      ← M4.3+M4.5 ★ 集成低电中断与恢复的主树
│   ├── patrol_round.xml                     ← M4.5   修改：支持断点恢复
│   └── charge_recovery.xml                  ← M4.4 ★ 回充子树
├── tests/
│   ├── test_battery_simulator.cpp           ← M4.1   BatterySimulatorNode 单元测试
│   └── test_battery_bt_nodes.cpp            ← M4.2-5 电池 BT 节点集成测试
└── readme_M4.md                             ← 本文件
```

---

## 架构设计

### 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│  BatterySimulatorNode (独立 ROS 节点)                        │
│  参数: discharge_rate, charge_rate, initial_percentage       │
│  发布: /battery_state (sensor_msgs/BatteryState) @ 1Hz       │
│  服务: /battery_simulator/set_charging (std_srvs/SetBool)    │
└──────────────────────────┬──────────────────────────────────┘
                           │ 订阅
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  BatteryMonitorNode (BT::ConditionNode)                      │
│  功能: 比较电量 vs 阈值，写入黑板 is_low_battery              │
│  返回值: SUCCESS(电量正常) / FAILURE(低电)                    │
│  黑板输出: battery_level, is_low_battery                     │
└──────────────────────────┬──────────────────────────────────┘
                           │ 触发
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  patrol_main.xml (ReactiveSequence)                          │
│  ┌─ BatteryMonitorNode (条件)                               │
│  └─ Selector                                                │
│       ├─ IsBatteryLowCondition → ChargeRecovery 子树        │
│       └─ PatrolRound 子树                                   │
└─────────────────────────────────────────────────────────────┘
```

### 低电中断与恢复流程

```
正常巡逻:
  BatteryMonitorNode → SUCCESS → Selector → PatrolRound
                                          ↓
                                   NavigateToPose → Wait → NextWaypoint

低电触发:
  BatteryMonitorNode → FAILURE → ReactiveSequence halt 巡逻
                                → Selector → IsBatteryLowCondition(true)
                                            → ChargeRecovery 子树:
                                                1. 保存 recovery_point_index
                                                2. NavigateToPose(charge_standby)
                                                3. NavigateToPose(charge_dock)
                                                4. SetChargingModeNode(enabled=true)
                                                5. Wait(10s) ← 模拟充电
                                                6. SetChargingModeNode(enabled=false)
                                                7. 重置 is_low_battery=false

恢复巡逻:
  BatteryMonitorNode → SUCCESS → Selector → PatrolRound
                                          ↓
                                  PatrolRound 重新开始:
                                  1. remaining = waypoints_count - current_index
                                  2. Repeat(remaining 次) ← 从断点继续
```

### 黑板键汇总

| 黑板键 | 类型 | 写入者 | 说明 |
|--------|------|--------|------|
| `battery_threshold` | float | Script | 低电阈值（%，默认 20.0） |
| `battery_level` | float | BatteryMonitorNode | 当前电量 [0, 100] |
| `is_low_battery` | bool | BatteryMonitorNode | 低电标志 |
| `recovery_point_index` | int | Script/ChargeRecovery | 回充断点索引 |
| `current_index` | int | NextWaypointNode | 当前巡逻索引 |

---

## 各任务说明

### M4.1 BatterySimulatorNode

独立的 ROS2 节点，发布电池状态消息。

- 参数：
  - `initial_percentage` (double, 默认 100.0)：初始电量
  - `discharge_rate` (double, 默认 0.5)：放电速率（%/秒）
  - `charge_rate` (double, 默认 5.0)：充电速率（%/秒）
  - `publish_rate` (double, 默认 1.0)：发布频率（Hz）
  - `voltage_full` / `voltage_empty` (double)：满电/空电电压
- 服务：`/battery_simulator/set_charging` (std_srvs/SetBool)
- 发布：`/battery_state` (sensor_msgs/BatteryState)

### M4.2 BatteryMonitorNode

继承 `BT::ConditionNode`，订阅 `/battery_state` 话题。

- 输入端口：`battery_threshold` (float)
- 输出端口：`battery_level` (float), `is_low_battery` (bool)
- 返回 SUCCESS（电量 ≥ 阈值）或 FAILURE（电量 < 阈值）
- 订阅内部自动创建，需要 `rclcpp::Node::SharedPtr`

### M4.3 低电中断逻辑

在 `patrol_main.xml` 中使用 `ReactiveSequence`：
- 第一个子节点为 `BatteryMonitorNode`（条件）
- 当条件返回 FAILURE 时，ReactiveSequence 自动 halt 正在运行的巡逻子节点
- `IsBatteryLowCondition` 根据黑板标志选择回充或巡逻分支

### M4.4 ChargeRecovery 子树

`behavior_trees/charge_recovery.xml` 包含完整回充流程：
1. 保存 `recovery_point_index`（当前巡逻索引）
2. `NavigateToPoseNode` → `charge_standby`
3. `NavigateToPoseNode` → `charge_dock`
4. `SetChargingModeNode` → 启用充电
5. `Wait` → 10 秒（模拟充电时间）
6. `SetChargingModeNode` → 禁用充电
7. 重置 `is_low_battery` = false, `battery_level` = 100

### M4.5 恢复巡逻逻辑

- `PatrolRound` 子树不再初始化 `current_index`（由主树管理）
- 入口处计算 `remaining = waypoints_count - current_index`
- `Repeat` 循环使用 `remaining` 而非固定 `waypoints_count`
- 充电完成后 `current_index` 从 `recovery_point_index` 恢复

---

## 运行测试

```bash
cd /home/zzk/PatrolBot-Indoor/patrol_robot_ws
colcon build --packages-select nav2_demo
source install/setup.bash

# M4 全部测试
colcon test --packages-select nav2_demo \
  --ctest-args -R "test_battery_simulator|test_battery_bt_nodes"

# 查看测试结果
colcon test-result --all

# M2+M3+M4 全部测试
colcon test --packages-select nav2_demo
```

---

## 测试覆盖

| 测试文件 | 模块 | 用例数 | 说明 |
|----------|------|--------|------|
| `test_battery_simulator.cpp` | M4.1 | 8 | 初始电量、放电、充电、边界钳制、服务接口、消息格式 |
| `test_battery_bt_nodes.cpp` | M4.2-5 | 12+ | BatteryMonitorNode(3) + IsBatteryLowCondition(3) + 集成(3) + 恢复(1) + SetChargingModeNode(2) |

---

## 启动指南

```bash
# 1. 启动电池模拟节点
ros2 run nav2_demo battery_simulator_node

# 2. 查看电池状态
ros2 topic echo /battery_state

# 3. 手动控制充电
ros2 service call /battery_simulator/set_charging std_srvs/srv/SetBool \
  "{data: true}"

# 4. 完整仿真 + 巡逻 + 电池（需集成到 launch 文件）
# 在 patrol_sim.launch.py 中添加电池模拟节点启动
```

---

## 模块依赖

| 依赖包 | 用途 |
|--------|------|
| `sensor_msgs` | BatteryState 消息定义 |
| `std_srvs` | SetBool 服务（充电控制） |
| `behaviortree_cpp` | BehaviorTree.CPP v4 核心库 |
