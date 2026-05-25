# 02. 配置加载器 (ConfigLoader)

> 对应需求: FR-01 路线定义与加载
> 参考设计: detailed_design.md §2.2, §6

## 2.1 头文件

- [ ] 创建 `include/patrol_bot/config_loader.hpp`
- [ ] 声明 `ConfigLoader` 类
- [ ] 声明静态方法 `Config load(const std::string& yaml_path)`
- [ ] 声明私有方法 `parse_route()` / `parse_waypoint()` / `parse_alarm()`

## 2.2 YAML 加载与顶层解析

- [ ] 创建 `src/config_loader.cpp`
- [ ] 实现 `load()` 入口：调用 `YAML::LoadFile()` 加载 YAML
- [ ] 实现文件不存在时抛 `std::runtime_error("Config file not found: " + path)`
- [ ] 解析 `nav2.waypoint_timeout` → `config.waypoint_timeout`

## 2.3 充电站解析

- [ ] 解析 `charging_station.x` / `.y` / `.yaw`
- [ ] 字段缺失时抛 `std::runtime_error("Missing required field: charging_station.xxx")`

## 2.4 电池配置解析

- [ ] 解析 `battery.initial_level`
- [ ] 解析 `battery.low_threshold`
- [ ] 解析 `battery.recovery_threshold`
- [ ] 解析 `battery.discharge_rate.moving`（不解析 idle_rate）
- [ ] 解析 `battery.charge_rate`
- [ ] 字段缺失时抛异常

## 2.5 摄像头配置解析（可选字段）

- [ ] 检查 `camera` 段是否存在
- [ ] 解析 `camera.topic`（默认 "/camera/image_raw"）
- [ ] 解析 `camera.image_format`（默认 "png"）
- [ ] 解析 `camera.save_directory`（默认 "images"）

## 2.6 路线解析

- [ ] 解析 `patrol_routes` 数组（非空校验）
- [ ] 实现 `parse_route()`：解析 name / priority / waypoints 数组
- [ ] 实现 `parse_waypoint()`：解析 x / y / yaw / wait_seconds
- [ ] 实现 `parse_alarm()`：解析 alarm_simulate 的 type / severity，返回 `std::optional<AlarmConfig>`
- [ ] waypoints 数组非空校验

## 2.7 错误处理

- [ ] 类型不匹配时捕获 `YAML::BadConversion`，包装为 `std::runtime_error`
- [ ] 确保所有必填字段缺失时都有清晰的错误消息

## 2.8 配置文件

- [ ] 创建 `config/patrol_config.yaml`（包含充电桩、电池、摄像头、路线、nav2 参数）

## 2.9 编译验证

- [ ] 依赖 yaml-cpp 库正确链接
- [ ] 编译通过，加载示例 YAML 正确
