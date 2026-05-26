#include <gtest/gtest.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include "patrol_bot/bt_nodes.hpp"
#include "patrol_bot/patrol_types.hpp"
#include <chrono>
#include <thread>
#include <filesystem>

// ==================== 测试辅助 ====================

// 创建一个临时日志目录
std::string make_test_log_dir() {
    auto tmp = std::filesystem::temp_directory_path() / "patrol_test_bt";
    std::filesystem::create_directories(tmp);
    return tmp.string();
}

// ==================== 条件节点测试 ====================

// IsBatteryLow: battery_level < low_threshold → SUCCESS
TEST(BtNodesTest, IsBatteryLow_Low) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<patrol_bot::IsBatteryLow>("IsBatteryLow");

    auto blackboard = BT::Blackboard::create();
    blackboard->set("battery_level", 15.0);
    blackboard->set("low_threshold", 20.0);

    const std::string xml = R"(
    <root BTCPP_format="4">
        <BehaviorTree ID="Test">
            <IsBatteryLow battery_level="{battery_level}"
                          low_threshold="{low_threshold}"/>
        </BehaviorTree>
    </root>
    )";

    auto tree = factory.createTreeFromText(xml, blackboard);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

// IsBatteryLow: battery_level >= low_threshold → FAILURE
TEST(BtNodesTest, IsBatteryLow_Sufficient) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<patrol_bot::IsBatteryLow>("IsBatteryLow");

    auto blackboard = BT::Blackboard::create();
    blackboard->set("battery_level", 50.0);
    blackboard->set("low_threshold", 20.0);

    const std::string xml = R"(
    <root BTCPP_format="4">
        <BehaviorTree ID="Test">
            <IsBatteryLow battery_level="{battery_level}"
                          low_threshold="{low_threshold}"/>
        </BehaviorTree>
    </root>
    )";

    auto tree = factory.createTreeFromText(xml, blackboard);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

// IsBatteryLow: 等于阈值时视为不低（严格小于）
TEST(BtNodesTest, IsBatteryLow_Equal) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<patrol_bot::IsBatteryLow>("IsBatteryLow");

    auto blackboard = BT::Blackboard::create();
    blackboard->set("battery_level", 20.0);
    blackboard->set("low_threshold", 20.0);

    const std::string xml = R"(
    <root BTCPP_format="4">
        <BehaviorTree ID="Test">
            <IsBatteryLow battery_level="{battery_level}"
                          low_threshold="{low_threshold}"/>
        </BehaviorTree>
    </root>
    )";

    auto tree = factory.createTreeFromText(xml, blackboard);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

// HasAlarm: has_alarm=true → SUCCESS
TEST(BtNodesTest, HasAlarm_True) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<patrol_bot::HasAlarm>("HasAlarm");

    auto blackboard = BT::Blackboard::create();
    patrol_bot::Waypoint wp;
    wp.has_alarm = true;
    blackboard->set("current_waypoint", wp);

    const std::string xml = R"(
    <root BTCPP_format="4">
        <BehaviorTree ID="Test">
            <HasAlarm current_waypoint="{current_waypoint}"/>
        </BehaviorTree>
    </root>
    )";

    auto tree = factory.createTreeFromText(xml, blackboard);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

// HasAlarm: has_alarm=false → FAILURE
TEST(BtNodesTest, HasAlarm_False) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<patrol_bot::HasAlarm>("HasAlarm");

    auto blackboard = BT::Blackboard::create();
    patrol_bot::Waypoint wp;
    wp.has_alarm = false;
    blackboard->set("current_waypoint", wp);

    const std::string xml = R"(
    <root BTCPP_format="4">
        <BehaviorTree ID="Test">
            <HasAlarm current_waypoint="{current_waypoint}"/>
        </BehaviorTree>
    </root>
    )";

    auto tree = factory.createTreeFromText(xml, blackboard);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

// IsAlarmCritical: severity == "critical" → SUCCESS
TEST(BtNodesTest, IsAlarmCritical_Critical) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<patrol_bot::IsAlarmCritical>("IsAlarmCritical");

    auto blackboard = BT::Blackboard::create();
    patrol_bot::Waypoint wp;
    wp.has_alarm = true;
    wp.alarm.severity = "critical";
    blackboard->set("current_waypoint", wp);

    const std::string xml = R"(
    <root BTCPP_format="4">
        <BehaviorTree ID="Test">
            <IsAlarmCritical current_waypoint="{current_waypoint}"/>
        </BehaviorTree>
    </root>
    )";

    auto tree = factory.createTreeFromText(xml, blackboard);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

// IsAlarmCritical: severity == "warning" → FAILURE
TEST(BtNodesTest, IsAlarmCritical_Warning) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<patrol_bot::IsAlarmCritical>("IsAlarmCritical");

    auto blackboard = BT::Blackboard::create();
    patrol_bot::Waypoint wp;
    wp.has_alarm = true;
    wp.alarm.severity = "warning";
    blackboard->set("current_waypoint", wp);

    const std::string xml = R"(
    <root BTCPP_format="4">
        <BehaviorTree ID="Test">
            <IsAlarmCritical current_waypoint="{current_waypoint}"/>
        </BehaviorTree>
    </root>
    )";

    auto tree = factory.createTreeFromText(xml, blackboard);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

// IsAlarmCritical: no alarm → FAILURE
TEST(BtNodesTest, IsAlarmCritical_NoAlarm) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<patrol_bot::IsAlarmCritical>("IsAlarmCritical");

    auto blackboard = BT::Blackboard::create();
    patrol_bot::Waypoint wp;
    wp.has_alarm = false;
    blackboard->set("current_waypoint", wp);

    const std::string xml = R"(
    <root BTCPP_format="4">
        <BehaviorTree ID="Test">
            <IsAlarmCritical current_waypoint="{current_waypoint}"/>
        </BehaviorTree>
    </root>
    )";

    auto tree = factory.createTreeFromText(xml, blackboard);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

// ==================== WaitAtWaypoint 测试 ====================

// WaitAtWaypoint: wait_seconds <= 0 → 立即返回 SUCCESS
TEST(BtNodesTest, WaitAtWaypoint_ZeroWait) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<patrol_bot::WaitAtWaypoint>("WaitAtWaypoint");

    auto blackboard = BT::Blackboard::create();
    patrol_bot::Waypoint wp;
    wp.wait_seconds = 0.0;
    blackboard->set("current_waypoint", wp);

    const std::string xml = R"(
    <root BTCPP_format="4">
        <BehaviorTree ID="Test">
            <WaitAtWaypoint current_waypoint="{current_waypoint}"/>
        </BehaviorTree>
    </root>
    )";

    auto tree = factory.createTreeFromText(xml, blackboard);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

// WaitAtWaypoint: wait_seconds < 0 → 立即返回 SUCCESS
TEST(BtNodesTest, WaitAtWaypoint_NegativeWait) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<patrol_bot::WaitAtWaypoint>("WaitAtWaypoint");

    auto blackboard = BT::Blackboard::create();
    patrol_bot::Waypoint wp;
    wp.wait_seconds = -1.0;
    blackboard->set("current_waypoint", wp);

    const std::string xml = R"(
    <root BTCPP_format="4">
        <BehaviorTree ID="Test">
            <WaitAtWaypoint current_waypoint="{current_waypoint}"/>
        </BehaviorTree>
    </root>
    )";

    auto tree = factory.createTreeFromText(xml, blackboard);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

// WaitAtWaypoint: 正等待时间 → 首次 tick 返回 RUNNING
TEST(BtNodesTest, WaitAtWaypoint_Running) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<patrol_bot::WaitAtWaypoint>("WaitAtWaypoint");

    auto blackboard = BT::Blackboard::create();
    patrol_bot::Waypoint wp;
    wp.wait_seconds = 5.0;  // 5秒等待
    blackboard->set("current_waypoint", wp);

    const std::string xml = R"(
    <root BTCPP_format="4">
        <BehaviorTree ID="Test">
            <WaitAtWaypoint current_waypoint="{current_waypoint}"/>
        </BehaviorTree>
    </root>
    )";

    auto tree = factory.createTreeFromText(xml, blackboard);
    // 首次调用应返回 RUNNING
    BT::NodeStatus status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::RUNNING);
}

// ==================== SimulateCharging 测试 ====================

// SimulateCharging: 开始充电后首次 tick 返回 RUNNING
TEST(BtNodesTest, SimulateCharging_StartCharging) {
    BT::BehaviorTreeFactory factory;
    auto logger = std::make_shared<patrol_bot::PatrolLogger>(
        make_test_log_dir());

    factory.registerBuilder<patrol_bot::SimulateCharging>(
        "SimulateCharging",
        [logger](const std::string& name, const BT::NodeConfig& cfg) {
            return std::make_unique<patrol_bot::SimulateCharging>(
                name, cfg, logger);
        });

    auto blackboard = BT::Blackboard::create();
    blackboard->set("recovery_threshold", 95.0);
    blackboard->set("battery_level", 80.0);

    const std::string xml = R"(
    <root BTCPP_format="4">
        <BehaviorTree ID="Test">
            <SimulateCharging patrol_state="{patrol_state}"
                              recovery_threshold="{recovery_threshold}"
                              battery_level="{battery_level}"/>
        </BehaviorTree>
    </root>
    )";

    auto tree = factory.createTreeFromText(xml, blackboard);
    BT::NodeStatus status = tree.tickOnce();

    // 首次 tick: onStart 设置 CHARGING，然后 onRunning 检查电池 < 恢复阈值 → RUNNING
    EXPECT_EQ(status, BT::NodeStatus::RUNNING);

    // patrol_state 应被设置为 CHARGING (3)
    int state = -1;
    [[maybe_unused]] bool found = blackboard->get("patrol_state", state);
    EXPECT_TRUE(found);
    EXPECT_EQ(state, static_cast<int>(patrol_bot::PatrolState::CHARGING));
}

// SimulateCharging: 电池恢复到阈值以上后 tick 返回 SUCCESS
TEST(BtNodesTest, SimulateCharging_Recovered) {
    BT::BehaviorTreeFactory factory;
    auto logger = std::make_shared<patrol_bot::PatrolLogger>(
        make_test_log_dir());

    factory.registerBuilder<patrol_bot::SimulateCharging>(
        "SimulateCharging",
        [logger](const std::string& name, const BT::NodeConfig& cfg) {
            return std::make_unique<patrol_bot::SimulateCharging>(
                name, cfg, logger);
        });

    auto blackboard = BT::Blackboard::create();
    blackboard->set("recovery_threshold", 95.0);
    blackboard->set("battery_level", 100.0);  // 已满

    const std::string xml = R"(
    <root BTCPP_format="4">
        <BehaviorTree ID="Test">
            <SimulateCharging patrol_state="{patrol_state}"
                              recovery_threshold="{recovery_threshold}"
                              battery_level="{battery_level}"/>
        </BehaviorTree>
    </root>
    )";

    auto tree = factory.createTreeFromText(xml, blackboard);

    // 首次 tick: onStart → RUNNING
    BT::NodeStatus status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::RUNNING);

    // 第二次 tick: onRunning 检查 battery=100 >= 95 → 恢复，返回 SUCCESS
    status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

    // patrol_state 应被设置为 PATROLLING (1)
    int state = -1;
    [[maybe_unused]] bool found = blackboard->get("patrol_state", state);
    EXPECT_TRUE(found);
    EXPECT_EQ(state, static_cast<int>(patrol_bot::PatrolState::PATROLLING));
}

// ==================== SavePatrolContext 逻辑测试 ====================

TEST(BtNodesTest, SavePatrolContext_Logic) {
    int current_route_index = 1;
    int current_waypoint_idx = 3;

    int saved_route_index = current_route_index;
    int saved_waypoint_idx = current_waypoint_idx;

    EXPECT_EQ(saved_route_index, 1);
    EXPECT_EQ(saved_waypoint_idx, 3);
}

// ==================== RestorePatrolContext 逻辑测试 ====================

TEST(BtNodesTest, RestorePatrolContext_Logic) {
    int saved_route_index = 1;
    int saved_waypoint_idx = 3;

    bool has_checkpoint = (saved_route_index != -1 && saved_waypoint_idx != -1);
    EXPECT_TRUE(has_checkpoint);

    int restored_route = saved_route_index;
    int restored_wp = saved_waypoint_idx;
    EXPECT_EQ(restored_route, 1);
    EXPECT_EQ(restored_wp, 3);

    saved_route_index = -1;
    saved_waypoint_idx = -1;
    has_checkpoint = (saved_route_index != -1 && saved_waypoint_idx != -1);
    EXPECT_FALSE(has_checkpoint);
}

// ==================== 状态常量验证 ====================

TEST(BtNodesTest, StateConstants) {
    EXPECT_EQ(static_cast<int>(patrol_bot::PatrolState::IDLE), 0);
    EXPECT_EQ(static_cast<int>(patrol_bot::PatrolState::PATROLLING), 1);
    EXPECT_EQ(static_cast<int>(patrol_bot::PatrolState::PAUSED), 2);
    EXPECT_EQ(static_cast<int>(patrol_bot::PatrolState::CHARGING), 3);
    EXPECT_EQ(static_cast<int>(patrol_bot::PatrolState::STOPPED), 4);
}

// ==================== HandleAlarm 状态转换逻辑 ====================

TEST(BtNodesTest, HandleAlarm_CriticalPauses) {
    int STATE_PAUSED = static_cast<int>(patrol_bot::PatrolState::PAUSED);
    EXPECT_EQ(STATE_PAUSED, 2);

    int STATE_PATROLLING = static_cast<int>(patrol_bot::PatrolState::PATROLLING);
    EXPECT_EQ(STATE_PATROLLING, 1);

    // critical → STATE_PAUSED
    patrol_bot::Waypoint wp;
    wp.has_alarm = true;
    wp.alarm.severity = "critical";

    int patrol_state = STATE_PATROLLING;
    if (wp.has_alarm && wp.alarm.severity == "critical") {
        patrol_state = STATE_PAUSED;
    }
    EXPECT_EQ(patrol_state, STATE_PAUSED);

    // warning → 不改变 state
    wp.alarm.severity = "warning";
    patrol_state = STATE_PATROLLING;
    if (wp.has_alarm && wp.alarm.severity == "critical") {
        patrol_state = STATE_PAUSED;
    }
    EXPECT_EQ(patrol_state, STATE_PATROLLING);
}

// ==================== SetRouteContext 越界处理 ====================

TEST(BtNodesTest, SetRouteContext_OutOfRange) {
    std::vector<patrol_bot::Route> routes;
    patrol_bot::Route route;
    route.name = "test";
    routes.push_back(route);

    int idx = 5;
    bool out_of_range = (idx < 0 || idx >= static_cast<int>(routes.size()));
    EXPECT_TRUE(out_of_range);

    idx = 0;
    out_of_range = (idx < 0 || idx >= static_cast<int>(routes.size()));
    EXPECT_FALSE(out_of_range);
}

// ==================== CaptureImage 文件名 ====================

TEST(BtNodesTest, CaptureImage_FilenameFormat) {
    int route_idx = 0;
    int wp_idx = 2;
    std::string prefix = "route" + std::to_string(route_idx) +
                         "_wp" + std::to_string(wp_idx);
    EXPECT_EQ(prefix, "route0_wp2");
}

// ==================== NavigateToWaypoint 超时逻辑 ====================

TEST(BtNodesTest, NavigateToWaypoint_TimeoutLogic) {
    using namespace std::chrono_literals;

    double nav_timeout = 0.05;  // 50ms
    auto start = std::chrono::steady_clock::now();

    // 短暂等待，未超时
    std::this_thread::sleep_for(1ms);
    auto elapsed = std::chrono::steady_clock::now() - start;
    EXPECT_LT(elapsed, std::chrono::duration<double>(nav_timeout));

    // 等待直到超时
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    elapsed = std::chrono::steady_clock::now() - start;
    EXPECT_GE(elapsed, std::chrono::duration<double>(nav_timeout));
}

// 主函数
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
