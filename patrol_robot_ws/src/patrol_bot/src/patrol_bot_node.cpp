#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>

#include "patrol_bot/patrol_types.hpp"
#include "patrol_bot/config_loader.hpp"
#include "patrol_bot/patrol_logger.hpp"
#include "patrol_bot/battery_model.hpp"
#include "patrol_bot/alarm_manager.hpp"
#include "patrol_bot/nav2_action_client.hpp"
#include "patrol_bot/camera_buffer.hpp"
#include "patrol_bot/bt_nodes.hpp"
#include "patrol_bot_interfaces/msg/patrol_status.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <memory>
#include <string>
#include <stdexcept>

using namespace std::chrono_literals;

namespace patrol_bot {

class PatrolBotNode : public rclcpp::Node {
public:
    PatrolBotNode()
        : Node("patrol_bot_node") {

        // 获取包安装路径
        const std::string share_dir = ament_index_cpp::get_package_share_directory("patrol_bot");

        // --- 阶段 1: 参数声明 ---
        declare_parameter("config_path", share_dir + "/config/patrol_config.yaml");
        declare_parameter("mock_navigation", false);

        // --- 阶段 2: 配置加载 (fail-fast) ---
        try {
            std::string config_path = get_parameter("config_path").as_string();
            config_ = ConfigLoader::load(config_path);
            RCLCPP_INFO(get_logger(), "Config loaded from: %s", config_path.c_str());
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "Config load failed: %s", e.what());
            throw;
        }

        // --- 阶段 3: 核心模块初始化 ---
        logger_ = std::make_shared<PatrolLogger>("logs");
        camera_buffer_ = std::make_shared<CameraBuffer>(
            this, config_.camera.topic,
            config_.camera.save_directory, config_.camera.image_format);
        battery_model_ = std::make_shared<BatteryModel>(this, config_.battery);
        alarm_manager_ = std::make_shared<AlarmManager>(this, *logger_);
        nav_client_ = std::make_shared<Nav2ActionClient>(this);
        if (get_parameter("mock_navigation").as_bool()) {
            nav_client_->set_mock(true);
            RCLCPP_INFO(get_logger(), "Mock navigation enabled — Nav2 calls will return success immediately");
        }

        // --- 阶段 4: 等待 Nav2 Action Server ---
        RCLCPP_INFO(get_logger(), "Waiting for Nav2 action server...");
        if (!nav_client_->wait_for_server(10s)) {
            RCLCPP_WARN(get_logger(), "Nav2 action server not ready after 10s, retrying...");
            bool ready = false;
            for (int i = 0; i < 3; ++i) {
                if (nav_client_->wait_for_server(5s)) {
                    ready = true;
                    break;
                }
                RCLCPP_WARN(get_logger(), "Retry %d/3: Nav2 action server still not ready", i + 1);
            }
            if (!ready) {
                RCLCPP_FATAL(get_logger(), "Nav2 action server unavailable after retries");
                throw std::runtime_error("Nav2 action server not ready");
            }
        }
        RCLCPP_INFO(get_logger(), "Nav2 action server is ready");

        // --- 阶段 5: 黑板初始化 ---
        blackboard_ = BT::Blackboard::create();
        blackboard_->set("config", config_);
        blackboard_->set("patrol_state", static_cast<int>(PatrolState::PATROLLING));
        blackboard_->set("battery_level", config_.battery.initial_level);
        blackboard_->set("saved_route_index", -1);
        blackboard_->set("saved_waypoint_idx", -1);
        blackboard_->set("current_route_index", 0);
        blackboard_->set("current_waypoint_index", 0);
        blackboard_->set("low_threshold", config_.battery.low_threshold);
        blackboard_->set("recovery_threshold", config_.battery.recovery_threshold);
        blackboard_->set("charging_station", config_.charging_station);
        blackboard_->set("nav_timeout", config_.waypoint_timeout);
        blackboard_->set("patrol_routes", config_.routes);

        // --- 阶段 6: BT 工厂注册 ---
        BT::BehaviorTreeFactory factory;
        register_nodes(factory);

        // --- 阶段 7: BT 实例化 ---
        std::string bt_xml_path = share_dir + "/bt_xml/patrol_tree.xml";
        tree_ = std::make_unique<BT::Tree>(
            factory.createTreeFromFile(bt_xml_path, blackboard_));

        // --- 阶段 8: ROS2 接口 ---
        // Service Servers
        start_srv_ = create_service<std_srvs::srv::Trigger>(
            "/patrol/start_patrol",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                (void)request;
                handle_start(response);
            });

        pause_srv_ = create_service<std_srvs::srv::Trigger>(
            "/patrol/pause_patrol",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                (void)request;
                handle_pause(response);
            });

        resume_srv_ = create_service<std_srvs::srv::Trigger>(
            "/patrol/resume_patrol",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                (void)request;
                handle_resume(response);
            });

        stop_srv_ = create_service<std_srvs::srv::Trigger>(
            "/patrol/stop_patrol",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                (void)request;
                handle_stop(response);
            });

        // Topic Publisher
        status_pub_ = create_publisher<patrol_bot_interfaces::msg::PatrolStatus>(
            "/patrol/status", 10);

        // /patrol/battery 订阅
        battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
            "/patrol/battery", 10,
            [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
                battery_callback(msg);
            });

        // --- 阶段 9: 定时器 ---
        bt_timer_ = create_wall_timer(50ms, [this]() { bt_tick(); });
        status_timer_ = create_wall_timer(1s, [this]() { publish_status(); });

        // --- 阶段 10: 启动 BatteryModel ---
        battery_model_->start();

        RCLCPP_INFO(get_logger(), "PatrolBot node initialized. Waiting for start command...");
    }

    ~PatrolBotNode() {
        battery_model_->stop();
    }

private:
    // ==================== BT 节点注册 ====================

    void register_nodes(BT::BehaviorTreeFactory& factory) {
        // === Inline Conditions ===
        factory.registerSimpleCondition("IsNotStopped", [](BT::TreeNode& node) {
            const auto& const_node = node;
            auto bb = const_node.config().blackboard;
            int state = bb->get<int>("patrol_state");
            return state != static_cast<int>(PatrolState::STOPPED)
                       ? BT::NodeStatus::SUCCESS
                       : BT::NodeStatus::FAILURE;
        });

        factory.registerSimpleCondition("IsNotPaused", [](BT::TreeNode& node) {
            const auto& const_node = node;
            auto bb = const_node.config().blackboard;
            int state = bb->get<int>("patrol_state");
            return state != static_cast<int>(PatrolState::PAUSED)
                       ? BT::NodeStatus::SUCCESS
                       : BT::NodeStatus::FAILURE;
        });

        // === Auto-registered Conditions (no deps) ===
        factory.registerNodeType<IsBatteryLow>("IsBatteryLow");
        factory.registerNodeType<HasAlarm>("HasAlarm");
        factory.registerNodeType<IsAlarmCritical>("IsAlarmCritical");

        // === SyncAction Nodes (RegisterBuilder for logger injection) ===
        BT::NodeBuilder builder_restore_ctx =
            [this](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<RestorePatrolContext>(name, config, logger_);
            };
        factory.registerBuilder<RestorePatrolContext>("RestorePatrolContext", builder_restore_ctx);

        BT::NodeBuilder builder_set_route =
            [this](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<SetRouteContext>(name, config, logger_);
            };
        factory.registerBuilder<SetRouteContext>("SetRouteContext", builder_set_route);

        BT::NodeBuilder builder_save_ctx =
            [this](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<SavePatrolContext>(name, config, logger_);
            };
        factory.registerBuilder<SavePatrolContext>("SavePatrolContext", builder_save_ctx);

        // === SyncAction with Camera ===
        BT::NodeBuilder builder_capture =
            [this](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<CaptureImage>(name, config, camera_buffer_, logger_);
            };
        factory.registerBuilder<CaptureImage>("CaptureImage", builder_capture);

        // === SyncAction with Alarm ===
        BT::NodeBuilder builder_alarm =
            [this](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<HandleAlarm>(name, config, alarm_manager_, logger_);
            };
        factory.registerBuilder<HandleAlarm>("HandleAlarm", builder_alarm);

        // === StatefulAction Nodes ===
        BT::NodeBuilder builder_nav_wp =
            [this](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<NavigateToWaypoint>(name, config, nav_client_, logger_);
            };
        factory.registerBuilder<NavigateToWaypoint>("NavigateToWaypoint", builder_nav_wp);

        BT::NodeBuilder builder_nav_ch =
            [this](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<NavigateToCharger>(name, config, nav_client_, logger_);
            };
        factory.registerBuilder<NavigateToCharger>("NavigateToCharger", builder_nav_ch);

        BT::NodeBuilder builder_sim_charge =
            [this](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<SimulateCharging>(name, config, logger_);
            };
        factory.registerBuilder<SimulateCharging>("SimulateCharging", builder_sim_charge);

        // WaitAtWaypoint is auto-registered (no deps)
        factory.registerNodeType<WaitAtWaypoint>("WaitAtWaypoint");

        // Inline action: advance to next route after inner loop completes
        factory.registerSimpleAction("AdvanceToNextRoute",
            [](const BT::TreeNode& node) {
                auto bb = node.config().blackboard;
                int route_idx = bb->get<int>("current_route_index");
                auto routes = bb->get<std::vector<Route>>("patrol_routes");
                int num_routes = static_cast<int>(routes.size());
                int next_route = (route_idx + 1) % num_routes;
                bb->set<int>("current_route_index", next_route);
                bb->set<int>("current_waypoint_index", 0);
                return BT::NodeStatus::SUCCESS;
            });
    }

    // ==================== BT tick 定时器 ====================

    void bt_tick() {
        int state = 0;
        try {
            state = blackboard_->get<int>("patrol_state");
        } catch (...) {
            return;
        }
        if (state == static_cast<int>(PatrolState::STOPPED)) {
            return;
        }
        tree_->tickExactlyOnce();
    }

    // ==================== 状态发布 ====================

    void publish_status() {
        auto msg = patrol_bot_interfaces::msg::PatrolStatus();
        msg.state = blackboard_->get<int>("patrol_state");
        msg.current_route_index = blackboard_->get<int>("current_route_index");
        msg.current_waypoint_index = blackboard_->get<int>("current_waypoint_index");
        msg.battery_level = static_cast<float>(blackboard_->get<double>("battery_level"));
        status_pub_->publish(msg);
    }

    // ==================== Battery 回调 ====================

    void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        blackboard_->set("battery_level", msg->percentage * 100.0);
    }

    // ==================== Service 回调 ====================

    void handle_start(std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        int state = blackboard_->get<int>("patrol_state");
        if (state == static_cast<int>(PatrolState::IDLE)) {
            blackboard_->set("patrol_state", static_cast<int>(PatrolState::PATROLLING));
            logger_->info("STATE", "Patrol started. IDLE -> PATROLLING");
            response->success = true;
            response->message = "Patrol started";
        } else {
            response->success = false;
            response->message = "Cannot start: current state=" + std::to_string(state);
        }
    }

    void handle_pause(std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        int state = blackboard_->get<int>("patrol_state");
        if (state == static_cast<int>(PatrolState::PATROLLING)) {
            blackboard_->set("patrol_state", static_cast<int>(PatrolState::PAUSED));
            logger_->info("STATE", "Patrol paused. PATROLLING -> PAUSED");
            response->success = true;
            response->message = "Patrol paused";
        } else {
            response->success = false;
            response->message = "Cannot pause: current state=" + std::to_string(state);
        }
    }

    void handle_resume(std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        int state = blackboard_->get<int>("patrol_state");
        if (state == static_cast<int>(PatrolState::PAUSED)) {
            blackboard_->set("patrol_state", static_cast<int>(PatrolState::PATROLLING));
            logger_->info("STATE", "Patrol resumed. PAUSED -> PATROLLING");
            response->success = true;
            response->message = "Patrol resumed";
        } else {
            response->success = false;
            response->message = "Cannot resume: current state=" + std::to_string(state);
        }
    }

    void handle_stop(std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        int state = blackboard_->get<int>("patrol_state");
        if (state != static_cast<int>(PatrolState::STOPPED)) {
            blackboard_->set("patrol_state", static_cast<int>(PatrolState::STOPPED));
            logger_->info("STATE",
                          "Patrol stopped. State " + std::to_string(state) + " -> STOPPED");
            response->success = true;
            response->message = "Patrol stopped";
        } else {
            response->success = false;
            response->message = "Already stopped";
        }
    }

    // ==================== 成员变量 ====================

    Config config_;
    std::shared_ptr<PatrolLogger> logger_;
    std::shared_ptr<CameraBuffer> camera_buffer_;
    std::shared_ptr<BatteryModel> battery_model_;
    std::shared_ptr<AlarmManager> alarm_manager_;
    std::shared_ptr<Nav2ActionClient> nav_client_;

    BT::Blackboard::Ptr blackboard_;
    std::unique_ptr<BT::Tree> tree_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;

    rclcpp::Publisher<patrol_bot_interfaces::msg::PatrolStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

    rclcpp::TimerBase::SharedPtr bt_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace patrol_bot

// ==================== main ====================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<patrol_bot::PatrolBotNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("PatrolBotNode"),
                     "Fatal error during initialization: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
