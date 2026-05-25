#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include "patrol_bot/patrol_types.hpp"
#include "patrol_bot/patrol_logger.hpp"
#include "patrol_bot/nav2_action_client.hpp"
#include "patrol_bot/camera_buffer.hpp"
#include "patrol_bot/alarm_manager.hpp"
#include <memory>

namespace patrol_bot {

// ==================== 条件节点 ====================

class IsBatteryLow : public BT::ConditionNode {
public:
    IsBatteryLow(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

class HasAlarm : public BT::ConditionNode {
public:
    HasAlarm(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

class IsAlarmCritical : public BT::ConditionNode {
public:
    IsAlarmCritical(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

// ==================== SyncAction 节点 ====================

class LoadRoutes : public BT::SyncActionNode {
public:
    LoadRoutes(const std::string& name, const BT::NodeConfig& config,
               std::shared_ptr<PatrolLogger> logger);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    std::shared_ptr<PatrolLogger> logger_;
};

class RestorePatrolContext : public BT::SyncActionNode {
public:
    RestorePatrolContext(const std::string& name, const BT::NodeConfig& config,
                         std::shared_ptr<PatrolLogger> logger);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    std::shared_ptr<PatrolLogger> logger_;
};

class SetRouteContext : public BT::SyncActionNode {
public:
    SetRouteContext(const std::string& name, const BT::NodeConfig& config,
                    std::shared_ptr<PatrolLogger> logger);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    std::shared_ptr<PatrolLogger> logger_;
};

class CaptureImage : public BT::SyncActionNode {
public:
    CaptureImage(const std::string& name, const BT::NodeConfig& config,
                 std::shared_ptr<CameraBuffer> camera_buffer,
                 std::shared_ptr<PatrolLogger> logger);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    std::shared_ptr<CameraBuffer> camera_buffer_;
    std::shared_ptr<PatrolLogger> logger_;
};

class HandleAlarm : public BT::SyncActionNode {
public:
    HandleAlarm(const std::string& name, const BT::NodeConfig& config,
                std::shared_ptr<AlarmManager> alarm_manager,
                std::shared_ptr<PatrolLogger> logger);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    std::shared_ptr<AlarmManager> alarm_manager_;
    std::shared_ptr<PatrolLogger> logger_;
};

class SavePatrolContext : public BT::SyncActionNode {
public:
    SavePatrolContext(const std::string& name, const BT::NodeConfig& config,
                      std::shared_ptr<PatrolLogger> logger);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    std::shared_ptr<PatrolLogger> logger_;
};

// ==================== StatefulAction 节点 ====================

class NavigateToWaypoint : public BT::StatefulActionNode {
public:
    NavigateToWaypoint(const std::string& name, const BT::NodeConfig& config,
                       std::shared_ptr<Nav2ActionClient> nav2_client,
                       std::shared_ptr<PatrolLogger> logger);
    static BT::PortsList providedPorts();

protected:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<Nav2ActionClient> nav2_client_;
    std::shared_ptr<PatrolLogger> logger_;
    std::chrono::steady_clock::time_point start_time_;
    double nav_timeout_ = 120.0;
};

class NavigateToCharger : public BT::StatefulActionNode {
public:
    NavigateToCharger(const std::string& name, const BT::NodeConfig& config,
                      std::shared_ptr<Nav2ActionClient> nav2_client,
                      std::shared_ptr<PatrolLogger> logger);
    static BT::PortsList providedPorts();

protected:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<Nav2ActionClient> nav2_client_;
    std::shared_ptr<PatrolLogger> logger_;
};

class WaitAtWaypoint : public BT::StatefulActionNode {
public:
    WaitAtWaypoint(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();

protected:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::chrono::steady_clock::time_point start_time_;
    double wait_duration_ = 0.0;
};

class SimulateCharging : public BT::StatefulActionNode {
public:
    SimulateCharging(const std::string& name, const BT::NodeConfig& config,
                     std::shared_ptr<PatrolLogger> logger);
    static BT::PortsList providedPorts();

protected:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<PatrolLogger> logger_;
    double recovery_threshold_ = 95.0;
};

}  // namespace patrol_bot
