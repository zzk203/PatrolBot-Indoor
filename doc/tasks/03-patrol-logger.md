# 03. 巡逻日志 (PatrolLogger)

> 对应需求: FR-12 日志内容、FR-13 日志输出
> 参考设计: detailed_design.md §2.5, §6.3

## 3.1 头文件

- [ ] 创建 `include/patrol_bot/patrol_logger.hpp`
- [ ] 声明 `PatrolLogger` 类
- [ ] 声明构造函数 `PatrolLogger(const std::string& log_dir)`
- [ ] 声明析构函数 `~PatrolLogger()`
- [ ] 声明方法 `info()` / `warn()` / `error()`
- [ ] 声明私有方法 `format()` / `write()` / `timestamp()`
- [ ] 声明成员变量 `std::ofstream file_` / `std::mutex mutex_`

## 3.2 实现

- [ ] 创建 `src/patrol_logger.cpp`
- [ ] 实现构造函数：`std::filesystem::create_directories(log_dir)` 创建日志目录
- [ ] 实现日志文件名生成：`patrol_YYYY-MM-DD_HH-MM-SS.log`（基于系统当前时间）
- [ ] 实现文件打开逻辑，失败抛 `std::runtime_error("Failed to create log file")`
- [ ] 实现析构函数：`close()` 关闭文件
- [ ] 实现 `timestamp()` 方法：返回 `"YYYY-MM-DD HH:MM:SS"` 格式字符串
- [ ] 实现 `format()` 方法：拼接 `[时间戳] [级别] [标签] 消息`
- [ ] 实现 `write()` 方法：
  - `std::lock_guard<std::mutex>` 保护写入
  - `std::cout << line << std::endl` 控制台输出
  - `file_ << line << std::endl` 文件输出
  - `file_.flush()` 立即落盘

## 3.3 日志级别

- [ ] 实现 `info(tag, msg)`：调用 `write("INFO", ...)`
- [ ] 实现 `warn(tag, msg)`：调用 `write("WARN", ...)`
- [ ] 实现 `error(tag, msg)`：调用 `write("ERROR", ...)`

## 3.4 编译验证

- [ ] 编译通过，日志文件创建正确
- [ ] 验证多线程并发写入安全性
