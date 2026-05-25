#include "patrol_bot/patrol_logger.hpp"
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>

namespace patrol_bot {

PatrolLogger::PatrolLogger(const std::string& log_dir) {
    // 确保目录存在
    std::filesystem::create_directories(log_dir);

    // 生成文件名
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t_now);

    std::ostringstream filename;
    filename << log_dir << "/patrol_"
             << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S")
             << ".log";

    file_.open(filename.str());
    if (!file_.is_open()) {
        throw std::runtime_error("Failed to create log file: " + filename.str());
    }
}

PatrolLogger::~PatrolLogger() {
    if (file_.is_open()) {
        file_.close();
    }
}

void PatrolLogger::info(const std::string& tag, const std::string& message) {
    write("INFO", tag, message);
}

void PatrolLogger::warn(const std::string& tag, const std::string& message) {
    write("WARN", tag, message);
}

void PatrolLogger::error(const std::string& tag, const std::string& message) {
    write("ERROR", tag, message);
}

void PatrolLogger::write(const std::string& level,
                         const std::string& tag,
                         const std::string& message) {
    std::string line = format(level, tag, message);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::cout << line << std::endl;
        file_ << line << std::endl;
        file_.flush();
    }
}

std::string PatrolLogger::format(const std::string& level,
                                  const std::string& tag,
                                  const std::string& message) {
    return "[" + timestamp() + "] [" + level + "] [" + tag + "] " + message;
}

std::string PatrolLogger::timestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t_now);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

}  // namespace patrol_bot
