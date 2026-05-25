#pragma once

#include "patrol_bot/patrol_types.hpp"
#include <yaml-cpp/yaml.h>
#include <string>
#include <optional>

namespace patrol_bot {

class ConfigLoader {
public:
    // 从 YAML 文件加载配置，解析失败抛出 std::runtime_error
    // @param yaml_path YAML 配置文件路径
    // @return 完整的 Config 结构体
    // @throws std::runtime_error 文件不存在、必填字段缺失、类型不匹配时
    static Config load(const std::string& yaml_path);

private:
    static Route parse_route(const YAML::Node& node);
    static Waypoint parse_waypoint(const YAML::Node& node);
    static std::optional<AlarmConfig> parse_alarm(const YAML::Node& node);
};

}  // namespace patrol_bot
