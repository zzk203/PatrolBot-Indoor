# M6: 轮次调度与完整任务编排

## 概述

M6 模块实现了机器人巡逻任务的完整轮次调度。在 M3-M5 的巡逻、低电回充、视觉对接基础上，M6 整合了"巡逻 → 返回待命点 → 等待间隔 → 重新开始"的完整闭环流程，并支持断点恢复和轮次重置。

## 架构设计

```
┌──────────────────────────────────────────────────────────────────────┐
│                     PatrolMain 主行为树 (M6.4)                         │
│  ┌──────────────────────────────────────────────────────────────────┐│
│  │  Sequence (初始化)                                               ││
│  │  ├─ Script: 设置参数 (battery_threshold, round_interval, ...)    ││
│  │  ├─ LoadWaypointsNode: 加载巡逻点配置                             ││
│  │  └─ Repeat(max_rounds)                                           ││
│  │       └─ Sequence(one_round)                                     ││
│  │            ├─ Phase 1: 内层巡逻循环 (M6.1 + M6.5)                ││
│  │            │  └─ Repeat(∞)                                       ││
│  │            │     └─ Sequence                                     ││
│  │            │        ├─ Inverter(IsPatrolCompleteCondition)       ││
│  │            │        └─ Fallback                                  ││
│  │            │           ├─ ReactiveSequence{Battery|PatrolRound}  ││
│  │            │           └─ Sequence{LowBattery|ChargeRecovery}    ││
│  │            ├─ Phase 2: ReturnToDock (M6.2)                      ││
│  │            ├─ Phase 3: Wait(round_interval) (M6.3)              ││
│  │            └─ Phase 4: Round Reset (M6.6)                       ││
│  │               ├─ current_index := 0                              ││
│  │               ├─ round_count++                                   ││
│  │               ├─ failure_count := 0                              ││
│  │               └─ patrol_complete := false                        ││
│  └──────────────────────────────────────────────────────────────────┘│
└──────────────────────────────────────────────────────────────────────┘
```

## 组件说明

### M6.1 轮次结束逻辑

- **IsPatrolCompleteCondition** (`BT::ConditionNode`): 新的条件节点，检查 `patrol_complete` 黑板标志
- **PatrolRound**: 所有巡逻点访问完后设置 `patrol_complete := true`，主树通过 `Inverter(IsPatrolCompleteCondition)` 在巡逻完成后跳出内层循环
- **端口**:
  - `patrol_complete` (input, bool): 巡逻完成标志

### M6.2 ReturnToDock

- 复用 `NavigateToPoseNode`，导航目标为 `dock_prep_pose` (即 `charge_standby`)
- 包装在 `Fallback{NavigateToPose | AlwaysSuccess}` 中：即使导航失败也继续后续流程（非关键步骤）
- 不执行视觉对接，仅导航到待命点

### M6.3 轮次间隔等待

- 使用 BT.CPP `Wait` 节点
- 间隔时间从黑板 `round_interval` 读取（默认 10 秒）
- 可通过 `Script` 修改或从配置文件加载

### M6.4 主行为树循环

完整的一轮流程：
1. **内层巡逻循环**: 反复尝试 PatrolRound，低电时自动回充，充完后从断点继续
2. **ReturnToDock**: 巡逻完成后导航到待命点
3. **Wait**: 等待轮次间隔
4. **Reset**: 索引归零、轮次计数递增、清除完成标志

### M6.5 断点续巡逻

- 低电时 `ReactiveSequence` 中断 `PatrolRound`
- `ChargeRecovery` 保存 `current_index` 到 `recovery_point_index`
- 充电完成后恢复 `current_index`
- 内层循环继续执行 `PatrolRound`（`remaining` 自动计算为剩余点数）
- 一轮内 `current_index` 持续递增，不被重置

### M6.6 轮次重置

一轮结束后执行：
- `current_index := 0`（下一轮从头巡逻）
- `round_count := round_count + 1`（轮次递增）
- `failure_count := 0`（失败计数清零）
- `patrol_complete := false`（清除完成标志）

## 新增/修改文件

| 文件 | 类型 | 说明 |
|------|------|------|
| `include/nav2_demo/bt_nodes/is_patrol_complete_condition.hpp` | 新增 | IsPatrolCompleteCondition 头文件 |
| `src/bt_nodes/is_patrol_complete_condition.cpp` | 新增 | IsPatrolCompleteCondition 实现 |
| `behavior_trees/patrol_round.xml` | 修改 | 完善轮次完成语义注释 |
| `behavior_trees/patrol_main.xml` | 重写 | 完整轮次调度主树 |
| `tests/test_m6_round_scheduling.cpp` | 新增 | 9 个单元测试 |
| `CMakeLists.txt` | 修改 | 新增源文件和测试目标 |
| `docs/checklist_M6.md` | 新增 | M6 检查清单 |
| `readme_M6.md` | 新增 | 本文件 |

## 测试策略

| 测试项 | 覆盖范围 |
|--------|---------|
| `IsPatrolCompleteConditionTest` (3 cases) | 条件节点 complete/not_complete/missing 三种场景 |
| `PatrolRoundCompletionTest` | 所有点访问完后完成标志设置 |
| `RoundResetTest` | 黑板变量重置的正确性 |
| `BreakpointResumeTest` | 从断点恢复后索引的正确性 |
| `PatrolLoopExitTest` | Inverter + 条件跳出循环的机制 |
| `ReturnToDockAndWaitTest` | 导航 + 等待的流程通过性 |
| `FullRoundCycleTest` | 完整一轮（巡逻→返回→等待→重置） |
| `BreakpointContinueTest` | 低电中断→充电→恢复→完成全流程 |
| `RoundIntervalWaitTest` | Wait 节点读取黑板参数 |

## 黑板参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `current_index` | int | 0 | 当前巡逻点索引 |
| `patrol_complete` | bool | false | 巡逻完成标志 |
| `round_count` | int | 0 | 已完成的轮次数 |
| `round_interval` | double | 10.0 | 轮次间隔等待时间（秒） |
| `max_rounds` | int | 1000000 | 最大轮次数 |
| `failure_count` | int | 0 | 累计失败次数 |
| `recovery_point_index` | int | -1 | 充电恢复断点索引 |
| `dock_prep_pose` | PoseStamped | - | 对接预备位姿（charge_standby） |

## 行为树 XML 结构

### 完整主树 (patrol_main.xml)

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="PatrolMain">
    <Sequence name="root_sequence">
      <!-- 初始化 -->
      <Script code="..."/> ...
      <LoadWaypointsNode .../>
      
      <!-- 轮次循环 -->
      <Repeat num_cycles="{max_rounds}">
        <Sequence name="one_round">
          
          <!-- 内层巡逻循环 -->
          <Repeat num_cycles="1000000">
            <Sequence>
              <Inverter>
                <IsPatrolCompleteCondition patrol_complete="{patrol_complete}"/>
              </Inverter>
              <Fallback>
                <ReactiveSequence>
                  <BatteryMonitorNode .../>
                  <SubTree ID="PatrolRound"/>
                </ReactiveSequence>
                <Sequence>
                  <IsBatteryLowCondition .../>
                  <Script code="recovery_point_index := {current_index}"/>
                  <SubTree ID="ChargeRecovery"/>
                </Sequence>
              </Fallback>
            </Sequence>
          </Repeat>
          
          <!-- ReturnToDock -->
          <Fallback>
            <NavigateToPoseNode goal="{dock_prep_pose}" .../>
            <AlwaysSuccess/>
          </Fallback>
          
          <!-- 轮次间隔 -->
          <Wait wait_duration="{round_interval}"/>
          
          <!-- 轮次重置 -->
          <Script code="current_index := 0"/>
          <Script code="round_count := {round_count} + 1"/>
          <Script code="failure_count := 0"/>
          <Script code="patrol_complete := false"/>
        </Sequence>
      </Repeat>
    </Sequence>
  </BehaviorTree>
</root>
```
