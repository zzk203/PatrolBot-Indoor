#include <gtest/gtest.h>
#include "patrol_bot/patrol_logger.hpp"
#include <fstream>
#include <filesystem>
#include <regex>
#include <sstream>
#include <thread>
#include <chrono>

namespace fs = std::filesystem;

class PatrolLoggerTest : public ::testing::Test {
protected:
    std::string tmp_dir;

    void SetUp() override {
        tmp_dir = (fs::temp_directory_path() / "patrol_test_log").string();
        fs::remove_all(tmp_dir);
        fs::create_directories(tmp_dir);
    }

    void TearDown() override {
        fs::remove_all(tmp_dir);
    }

    /// @brief 读取目录中第一个 patrol_*.log 文件的全部内容
    std::string read_log_file(const std::string& dir) {
        for (const auto& entry : fs::directory_iterator(dir)) {
            auto fname = entry.path().filename().string();
            if (fname.find("patrol_") == 0 && fname.size() > 6) {
                std::ifstream f(entry.path());
                if (!f.is_open()) {
                    return "";
                }
                std::stringstream ss;
                ss << f.rdbuf();
                return ss.str();
            }
        }
        return "";
    }

    /// @brief 获取日志文件路径（仅当唯一时）
    std::string get_log_file_path(const std::string& dir) {
        for (const auto& entry : fs::directory_iterator(dir)) {
            auto fname = entry.path().filename().string();
            if (fname.find("patrol_") == 0 && fname.size() > 6) {
                return entry.path().string();
            }
        }
        return "";
    }
};

// ============================================================
// 1. 文件创建
// ============================================================
TEST_F(PatrolLoggerTest, FileCreation) {
    // 构造 PatrolLogger，验证目录和文件被正确创建
    ASSERT_NO_THROW({
        patrol_bot::PatrolLogger logger(tmp_dir);
    });

    // 验证目录存在
    EXPECT_TRUE(fs::exists(tmp_dir));
    EXPECT_TRUE(fs::is_directory(tmp_dir));

    // 验证日志文件存在且文件名格式正确
    std::string log_file = get_log_file_path(tmp_dir);
    EXPECT_FALSE(log_file.empty()) << "日志文件未找到";

    // 验证文件名匹配 patrol_YYYY-MM-DD_HH-MM-SS.log
    std::regex file_pattern(R"(patrol_\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}\.log)");
    std::string fname = fs::path(log_file).filename().string();
    EXPECT_TRUE(std::regex_match(fname, file_pattern))
        << "文件名格式不匹配: " << fname;
}

// ============================================================
// 2. 多级别写入 — INFO
// ============================================================
TEST_F(PatrolLoggerTest, WriteInfo) {
    {
        patrol_bot::PatrolLogger logger(tmp_dir);
        logger.info("TestTag", "Hello info world");
    } // 析构关闭文件

    std::string content = read_log_file(tmp_dir);
    EXPECT_FALSE(content.empty());

    EXPECT_NE(content.find("[INFO]"), std::string::npos)
        << "日志中应包含 [INFO]";
    EXPECT_NE(content.find("[TestTag]"), std::string::npos)
        << "日志中应包含 [TestTag]";
    EXPECT_NE(content.find("Hello info world"), std::string::npos)
        << "日志中应包含消息内容";
}

// ============================================================
// 3. 多级别写入 — WARN
// ============================================================
TEST_F(PatrolLoggerTest, WriteWarn) {
    {
        patrol_bot::PatrolLogger logger(tmp_dir);
        logger.warn("WarnTag", "Warning message here");
    }

    std::string content = read_log_file(tmp_dir);
    EXPECT_FALSE(content.empty());

    EXPECT_NE(content.find("[WARN]"), std::string::npos)
        << "日志中应包含 [WARN]";
    EXPECT_NE(content.find("[WarnTag]"), std::string::npos)
        << "日志中应包含 [WarnTag]";
    EXPECT_NE(content.find("Warning message here"), std::string::npos)
        << "日志中应包含消息内容";
}

// ============================================================
// 4. 多级别写入 — ERROR
// ============================================================
TEST_F(PatrolLoggerTest, WriteError) {
    {
        patrol_bot::PatrolLogger logger(tmp_dir);
        logger.error("ErrTag", "Error occurred!");
    }

    std::string content = read_log_file(tmp_dir);
    EXPECT_FALSE(content.empty());

    EXPECT_NE(content.find("[ERROR]"), std::string::npos)
        << "日志中应包含 [ERROR]";
    EXPECT_NE(content.find("[ErrTag]"), std::string::npos)
        << "日志中应包含 [ErrTag]";
    EXPECT_NE(content.find("Error occurred!"), std::string::npos)
        << "日志中应包含消息内容";
}

// ============================================================
// 5. 时间戳格式验证
// ============================================================
TEST_F(PatrolLoggerTest, TimestampFormat) {
    {
        patrol_bot::PatrolLogger logger(tmp_dir);
        logger.info("TS", "check timestamp");
    }

    std::string content = read_log_file(tmp_dir);
    ASSERT_FALSE(content.empty());

    // 提取第一行的时间戳部分
    // 格式: [YYYY-MM-DD HH:MM:SS] [INFO] [TS] check timestamp
    std::regex ts_regex(R"(\[\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\])");
    std::smatch match;
    EXPECT_TRUE(std::regex_search(content, match, ts_regex))
        << "日志行中应包含 [YYYY-MM-DD HH:MM:SS] 格式的时间戳";

    if (match.size() > 0) {
        std::string ts = match[0].str();
        // 验证时间戳在行首
        EXPECT_EQ(content.find('['), 0u)
            << "时间戳应在行首";
        EXPECT_NE(content.find(ts), std::string::npos);
    }
}

// ============================================================
// 6. flush 验证 — 写入后立即读取文件验证内容已落盘
// ============================================================
TEST_F(PatrolLoggerTest, FlushVerify) {
    patrol_bot::PatrolLogger logger(tmp_dir);

    // 写入后不析构，直接读取文件验证内容已落盘
    logger.info("FlushTest", "immediate flush check");

    std::string content = read_log_file(tmp_dir);
    EXPECT_FALSE(content.empty()) << "写入后应立即落盘，文件不应为空";
    EXPECT_NE(content.find("immediate flush check"), std::string::npos)
        << "内容应已写入磁盘";
}

// ============================================================
// 7. 控制台输出验证
// ============================================================
TEST_F(PatrolLoggerTest, ConsoleOutput) {
    // 重定向 std::cout 的缓冲区来捕获控制台输出
    std::stringstream captured;

    // 保存原始缓冲区
    auto* old_buf = std::cout.rdbuf();
    std::cout.rdbuf(captured.rdbuf());

    {
        patrol_bot::PatrolLogger logger(tmp_dir);
        logger.info("ConsoleTag", "console message");
    }

    // 恢复原始缓冲区
    std::cout.rdbuf(old_buf);

    // 验证捕获的内容
    std::string console_out = captured.str();
    EXPECT_FALSE(console_out.empty()) << "控制台应有输出";
    EXPECT_NE(console_out.find("[INFO]"), std::string::npos)
        << "控制台输出应包含 [INFO]";
    EXPECT_NE(console_out.find("[ConsoleTag]"), std::string::npos)
        << "控制台输出应包含 [ConsoleTag]";
    EXPECT_NE(console_out.find("console message"), std::string::npos)
        << "控制台输出应包含消息内容";
}

// ============================================================
// 9. 多行日志验证 — 连续写入多条日志
// ============================================================
TEST_F(PatrolLoggerTest, MultipleLines) {
    {
        patrol_bot::PatrolLogger logger(tmp_dir);
        logger.info("ML", "line 1");
        logger.warn("ML", "line 2");
        logger.error("ML", "line 3");
    }

    std::string content = read_log_file(tmp_dir);
    ASSERT_FALSE(content.empty());

    // 按行分割
    std::vector<std::string> lines;
    std::stringstream ss(content);
    std::string line;
    while (std::getline(ss, line)) {
        if (!line.empty()) {
            lines.push_back(line);
        }
    }

    ASSERT_EQ(lines.size(), 3u) << "应写入 3 行日志";
    EXPECT_NE(lines[0].find("line 1"), std::string::npos);
    EXPECT_NE(lines[1].find("line 2"), std::string::npos);
    EXPECT_NE(lines[2].find("line 3"), std::string::npos);
}

// ============================================================
// 10. 特殊字符 — 标签和消息含特殊字符
// ============================================================
TEST_F(PatrolLoggerTest, SpecialCharacters) {
    {
        patrol_bot::PatrolLogger logger(tmp_dir);
        logger.info("Tag_123", "Message with special chars: !@#$%^&*()");
        logger.warn("Tag with spaces", "message with\nnewline");
    }

    std::string content = read_log_file(tmp_dir);
    EXPECT_FALSE(content.empty());
    EXPECT_NE(content.find("!@#$%^&*()"), std::string::npos);
    EXPECT_NE(content.find("Tag with spaces"), std::string::npos);
}

// ============================================================
// 11. 并发安全性基础 — 快速连续调用
// ============================================================
TEST_F(PatrolLoggerTest, SequentialFastCalls) {
    patrol_bot::PatrolLogger logger(tmp_dir);
    const int N = 100;
    for (int i = 0; i < N; ++i) {
        logger.info("Stress", "seq call " + std::to_string(i));
    }

    std::string content = read_log_file(tmp_dir);
    int count = 0;
    size_t pos = 0;
    while ((pos = content.find("seq call ", pos)) != std::string::npos) {
        ++count;
        ++pos;
    }
    EXPECT_EQ(count, N) << "应包含 " << N << " 条日志记录";
}

// ============================================================
// 12. 空消息和空标签
// ============================================================
TEST_F(PatrolLoggerTest, EmptyTagAndMessage) {
    {
        patrol_bot::PatrolLogger logger(tmp_dir);
        logger.info("", "");
        logger.error("EmptyTag", "");
        logger.warn("", "JustMessage");
    }

    std::string content = read_log_file(tmp_dir);
    EXPECT_FALSE(content.empty());
    EXPECT_NE(content.find("[INFO]"), std::string::npos);
    EXPECT_NE(content.find("[ERROR]"), std::string::npos);
    EXPECT_NE(content.find("[WARN]"), std::string::npos);
    EXPECT_NE(content.find("JustMessage"), std::string::npos);
}
