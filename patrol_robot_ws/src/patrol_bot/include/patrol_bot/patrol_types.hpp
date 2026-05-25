#pragma once

#include <string>
#include <vector>
#include <optional>

namespace patrol_bot {

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct AlarmConfig {
    std::string type;
    std::string severity;
};

struct Waypoint {
    Pose2D pose;
    double wait_seconds = 0.0;
    bool has_alarm = false;
    AlarmConfig alarm;
};

struct Route {
    std::string name;
    int priority = 0;
    std::vector<Waypoint> waypoints;
};

struct BatteryConfig {
    double initial_level = 100.0;
    double low_threshold = 20.0;
    double recovery_threshold = 95.0;
    double moving_rate = 0.5;
    double charge_rate = 2.0;
};

struct CameraConfig {
    std::string topic = "/camera/image_raw";
    std::string image_format = "png";
    std::string save_directory = "images";
};

struct Config {
    Pose2D charging_station;
    BatteryConfig battery;
    std::vector<Route> routes;
    double waypoint_timeout = 120.0;
    CameraConfig camera;
};

enum class PatrolState {
    IDLE = 0,
    PATROLLING = 1,
    PAUSED = 2,
    CHARGING = 3,
    STOPPED = 4
};

}  // namespace patrol_bot
