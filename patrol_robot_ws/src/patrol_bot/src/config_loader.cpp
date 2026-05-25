#include "patrol_bot/config_loader.hpp"
#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <fstream>
#include <filesystem>

namespace patrol_bot {

Config ConfigLoader::load(const std::string& yaml_path) {
    // 1. 检查文件是否存在
    if (!std::filesystem::exists(yaml_path)) {
        throw std::runtime_error("Config file not found: " + yaml_path);
    }

    // 2. 加载 YAML
    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Failed to parse config file: " + std::string(e.what()));
    }

    Config config;

    // 3. 解析 nav2.waypoint_timeout（必填）
    if (!yaml["nav2"] || !yaml["nav2"]["waypoint_timeout"]) {
        throw std::runtime_error("Missing required field: nav2.waypoint_timeout");
    }
    config.waypoint_timeout = yaml["nav2"]["waypoint_timeout"].as<double>();
    if (config.waypoint_timeout <= 0) {
        throw std::runtime_error("nav2.waypoint_timeout must be > 0");
    }

    // 4. 解析充电站（必填）
    if (!yaml["charging_station"]) {
        throw std::runtime_error("Missing required field: charging_station");
    }
    auto cs = yaml["charging_station"];
    try {
        config.charging_station.x = cs["x"].as<double>();
    } catch (...) {
        throw std::runtime_error("Missing required field: charging_station.x");
    }
    try {
        config.charging_station.y = cs["y"].as<double>();
    } catch (...) {
        throw std::runtime_error("Missing required field: charging_station.y");
    }
    try {
        config.charging_station.yaw = cs["yaw"].as<double>();
    } catch (...) {
        throw std::runtime_error("Missing required field: charging_station.yaw");
    }

    // 5. 解析电池配置（必填）
    if (!yaml["battery"]) {
        throw std::runtime_error("Missing required field: battery");
    }
    auto bat = yaml["battery"];
    try {
        config.battery.initial_level = bat["initial_level"].as<double>();
    } catch (...) {
        throw std::runtime_error("Missing required field: battery.initial_level");
    }
    try {
        config.battery.low_threshold = bat["low_threshold"].as<double>();
    } catch (...) {
        throw std::runtime_error("Missing required field: battery.low_threshold");
    }
    try {
        config.battery.recovery_threshold = bat["recovery_threshold"].as<double>();
    } catch (...) {
        throw std::runtime_error("Missing required field: battery.recovery_threshold");
    }
    try {
        config.battery.moving_rate = bat["discharge_rate"]["moving"].as<double>();
    } catch (...) {
        throw std::runtime_error("Missing required field: battery.discharge_rate.moving");
    }
    try {
        config.battery.charge_rate = bat["charge_rate"].as<double>();
    } catch (...) {
        throw std::runtime_error("Missing required field: battery.charge_rate");
    }

    // 验证：low_threshold < recovery_threshold
    if (config.battery.low_threshold >= config.battery.recovery_threshold) {
        throw std::runtime_error("battery.low_threshold must be less than battery.recovery_threshold");
    }
    // 验证：initial_level 在 [0, 100]
    if (config.battery.initial_level < 0 || config.battery.initial_level > 100) {
        throw std::runtime_error("battery.initial_level must be in [0, 100]");
    }

    // 6. 解析摄像头配置（可选字段，有默认值）
    if (yaml["camera"]) {
        auto cam = yaml["camera"];
        if (cam["topic"]) {
            config.camera.topic = cam["topic"].as<std::string>();
        }
        if (cam["image_format"]) {
            config.camera.image_format = cam["image_format"].as<std::string>();
        }
        if (cam["save_directory"]) {
            config.camera.save_directory = cam["save_directory"].as<std::string>();
        }
    }

    // 7. 解析巡逻路线（必填，至少1条）
    if (!yaml["patrol_routes"]) {
        throw std::runtime_error("Missing required field: patrol_routes");
    }
    if (!yaml["patrol_routes"].IsSequence() || yaml["patrol_routes"].size() == 0) {
        throw std::runtime_error("patrol_routes must be a non-empty array");
    }
    for (const auto& route_node : yaml["patrol_routes"]) {
        config.routes.push_back(parse_route(route_node));
    }

    return config;
}

Route ConfigLoader::parse_route(const YAML::Node& node) {
    Route route;
    try {
        route.name = node["name"].as<std::string>();
    } catch (...) {
        throw std::runtime_error("Missing required field: route.name");
    }
    try {
        route.priority = node["priority"].as<int>();
    } catch (...) {
        throw std::runtime_error("Missing required field: route.priority");
    }

    if (!node["waypoints"] || !node["waypoints"].IsSequence() || node["waypoints"].size() == 0) {
        throw std::runtime_error("Route \"" + route.name + "\" must have non-empty waypoints");
    }

    for (const auto& wp_node : node["waypoints"]) {
        route.waypoints.push_back(parse_waypoint(wp_node));
    }
    return route;
}

Waypoint ConfigLoader::parse_waypoint(const YAML::Node& node) {
    Waypoint wp;
    try {
        wp.pose.x = node["x"].as<double>();
    } catch (...) {
        throw std::runtime_error("Missing required field: waypoint.x");
    }
    try {
        wp.pose.y = node["y"].as<double>();
    } catch (...) {
        throw std::runtime_error("Missing required field: waypoint.y");
    }
    try {
        wp.pose.yaw = node["yaw"].as<double>();
    } catch (...) {
        throw std::runtime_error("Missing required field: waypoint.yaw");
    }

    try {
        wp.wait_seconds = node["wait_seconds"].as<double>();
    } catch (...) {
        throw std::runtime_error("Missing required field: waypoint.wait_seconds");
    }

    // 解析可选的 alarm_simulate
    auto alarm_opt = parse_alarm(node["alarm_simulate"]);
    if (alarm_opt) {
        wp.has_alarm = true;
        wp.alarm = *alarm_opt;
    }

    return wp;
}

std::optional<AlarmConfig> ConfigLoader::parse_alarm(const YAML::Node& node) {
    if (!node || !node.IsMap()) {
        return std::nullopt;
    }
    AlarmConfig alarm;
    try {
        alarm.type = node["type"].as<std::string>();
    } catch (...) {
        return std::nullopt;
    }
    try {
        alarm.severity = node["severity"].as<std::string>();
    } catch (...) {
        return std::nullopt;
    }
    // 验证 severity 只能是 "warning" 或 "critical"
    if (alarm.severity != "warning" && alarm.severity != "critical") {
        throw std::runtime_error("Invalid alarm severity: " + alarm.severity + ". Must be 'warning' or 'critical'");
    }
    return alarm;
}

}  // namespace patrol_bot
