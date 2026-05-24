#include "nav2_demo/bt_nodes/load_waypoints_node.hpp"

#include <fstream>
#include <stdexcept>

#include "yaml-cpp/yaml.h"

namespace nav2_demo
{

LoadWaypointsNode::LoadWaypointsNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList LoadWaypointsNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("waypoints_file", "Path to waypoints YAML file"),
    BT::OutputPort<std::vector<Waypoint>>("patrol_waypoints", "Loaded patrol waypoints"),
    BT::OutputPort<Waypoint>("charge_standby", "Charge standby position"),
    BT::OutputPort<Waypoint>("charge_dock", "Charge dock position"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("dock_prep_pose", "Dock prep pose (from charge_standby)"),
    BT::OutputPort<double>("dock_pose_x", "Dock X coordinate"),
    BT::OutputPort<double>("dock_pose_y", "Dock Y coordinate"),
    BT::OutputPort<double>("dock_pose_yaw", "Dock yaw (radians)"),
    BT::OutputPort<int>("waypoints_count", "Number of patrol waypoints"),
    BT::OutputPort<double>("navigation_timeout", "Navigation timeout in seconds"),
    BT::OutputPort<double>("waypoint_wait_duration", "Wait duration at each waypoint"),
  };
}

BT::NodeStatus LoadWaypointsNode::tick()
{
  auto waypoints_file = getInput<std::string>("waypoints_file");
  if (!waypoints_file) {
    RCLCPP_ERROR(rclcpp::get_logger("LoadWaypointsNode"),
      "Missing required input port: waypoints_file");
    return BT::NodeStatus::FAILURE;
  }

  PatrolConfig config = parseYaml(waypoints_file.value());

  if (config.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("LoadWaypointsNode"),
      "No patrol points found in file: %s", waypoints_file.value().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // 写入黑板
  setOutput("patrol_waypoints", config.patrol_points);
  setOutput("charge_standby", config.charge_standby);
  setOutput("charge_dock", config.charge_dock);

  // M5.1: 输出 dock_prep_pose (PoseStamped) 由 charge_standby 转换
  setOutput("dock_prep_pose", config.charge_standby.toPoseStamped());
  setOutput("dock_pose_x", config.charge_dock.x);
  setOutput("dock_pose_y", config.charge_dock.y);
  setOutput("dock_pose_yaw", config.charge_dock.yaw);

  setOutput("waypoints_count", static_cast<int>(config.size()));
  setOutput("navigation_timeout", config.navigation_timeout);
  setOutput("waypoint_wait_duration", config.waypoint_wait_duration);

  RCLCPP_INFO(rclcpp::get_logger("LoadWaypointsNode"),
    "Loaded %zu patrol waypoints, timeout=%.1fs, wait=%.1fs",
    config.size(), config.navigation_timeout, config.waypoint_wait_duration);

  return BT::NodeStatus::SUCCESS;
}

PatrolConfig LoadWaypointsNode::parseYaml(const std::string & yaml_path)
{
  PatrolConfig config;

  std::ifstream fin(yaml_path);
  if (!fin.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("LoadWaypointsNode"),
      "Cannot open waypoints file: %s", yaml_path.c_str());
    return config;
  }

  YAML::Node root = YAML::Load(fin);

  // 解析巡逻点
  if (root["patrol_points"]) {
    for (const auto & node : root["patrol_points"]) {
      Waypoint wp;
      wp.x = node["x"].as<double>(0.0);
      wp.y = node["y"].as<double>(0.0);
      wp.yaw = node["yaw"].as<double>(0.0);
      config.patrol_points.push_back(wp);
    }
  }

  // 解析充电预备点
  if (root["charge_standby"]) {
    config.charge_standby.x = root["charge_standby"]["x"].as<double>(0.0);
    config.charge_standby.y = root["charge_standby"]["y"].as<double>(0.0);
    config.charge_standby.yaw = root["charge_standby"]["yaw"].as<double>(0.0);
  }

  // 解析充电桩
  if (root["charge_dock"]) {
    config.charge_dock.x = root["charge_dock"]["x"].as<double>(0.0);
    config.charge_dock.y = root["charge_dock"]["y"].as<double>(0.0);
    config.charge_dock.yaw = root["charge_dock"]["yaw"].as<double>(0.0);
  }

  // 解析超时和等待时间（可选）
  if (root["navigation_timeout"]) {
    config.navigation_timeout = root["navigation_timeout"].as<double>(60.0);
  }
  if (root["waypoint_wait_duration"]) {
    config.waypoint_wait_duration = root["waypoint_wait_duration"].as<double>(3.0);
  }

  return config;
}

}  // namespace nav2_demo
