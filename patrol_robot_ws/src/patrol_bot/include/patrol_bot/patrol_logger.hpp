#pragma once

#include <string>
#include <fstream>
#include <mutex>

namespace patrol_bot {

class PatrolLogger {
public:
    /// @brief 构造日志器，自动创建日志目录和文件
    /// @param log_dir 日志输出目录，文件自动命名为 patrol_YYYY-MM-DD_HH-MM-SS.log
    /// @throws std::runtime_error 日志目录或文件创建失败时
    explicit PatrolLogger(const std::string& log_dir);

    ~PatrolLogger();

    void info(const std::string& tag, const std::string& message);
    void warn(const std::string& tag, const std::string& message);
    void error(const std::string& tag, const std::string& message);

private:
    std::string format(const std::string& level,
                       const std::string& tag,
                       const std::string& message);
    void write(const std::string& level,
               const std::string& tag,
               const std::string& message);
    static std::string timestamp();

    std::ofstream file_;
    std::mutex mutex_;
};

}  // namespace patrol_bot
