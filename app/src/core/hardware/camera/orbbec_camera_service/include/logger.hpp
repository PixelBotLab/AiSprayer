#pragma once

#include <cerrno>
#include <fcntl.h>
#include <unistd.h>

#include <atomic>
#include <iostream>
#include <sstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <mutex>

namespace orbbec_service {

enum class LogLevel {
    DEBUG = 0,
    INFO,
    WARN,
    ERROR
};

class Logger {
public:
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    void setLogLevel(LogLevel level) {
        current_level_ = level;
    }

    void setSimpleFormat(bool simple) {
        simple_format_ = simple;
    }

    template <typename... Args>
    void log(LogLevel level, const std::string& tag, Args&&... args) {
        if (level < current_level_) {
            return;
        }

        // 先在内存里拼完整行，最后一次性写出：持锁期间不碰 fd，写失败也只丢这一行。
        std::string line;
        if (simple_format_) {
            // Simplified format for parent process ingestion: [I] [Camera] message...
            char lvl_char = 'I';
            switch (level) {
                case LogLevel::DEBUG: lvl_char = 'D'; break;
                case LogLevel::INFO:  lvl_char = 'I'; break;
                case LogLevel::WARN:  lvl_char = 'W'; break;
                case LogLevel::ERROR: lvl_char = 'E'; break;
            }
            std::ostringstream oss;
            oss << "[" << lvl_char << "] [" << tag << "] ";
            (oss << ... << std::forward<Args>(args));
            line = oss.str();
        } else {
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

            std::stringstream ss;
            ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S")
               << "." << std::setfill('0') << std::setw(3) << ms.count();

            std::string level_str;
            std::string color_code;
            const std::string reset_code = "\033[0m";

            switch (level) {
                case LogLevel::DEBUG:
                    level_str = "[DEBUG]";
                    color_code = "\033[36m"; // Cyan
                    break;
                case LogLevel::INFO:
                    level_str = "[INFO ]";
                    color_code = "\033[32m"; // Green
                    break;
                case LogLevel::WARN:
                    level_str = "[WARN ]";
                    color_code = "\033[33m"; // Yellow
                    break;
                case LogLevel::ERROR:
                    level_str = "[ERROR]";
                    color_code = "\033[31m"; // Red
                    break;
            }

            std::ostringstream oss;
            oss << color_code << level_str << " " << ss.str() << " [" << tag << "] " << reset_code;
            (oss << ... << std::forward<Args>(args));
            line = oss.str();
        }
        line.push_back('\n');

        std::lock_guard<std::mutex> lock(mutex_);
        writeOutNonBlocking(line);
    }

    long long droppedLines() const { return dropped_lines_.load(); }

private:
    // 日志输出绝不阻塞业务线程。stdout 是管道（Python 门面消费）或终端：消费端一旦停摆
    // （管道写满 / 终端网络断开），阻塞式 write() 会把调用它的线程整个冻住 —— 实测曾把
    // HTTP 线程冻住 90+ 秒，页面上就是"使能 follow 失败，后端服务无响应"（日志背压）。
    // 对策：stdout 一次性置 O_NONBLOCK；写不进（EAGAIN）就丢掉这一行并计数。stdout 被
    // 重定向到普通文件时 O_NONBLOCK 无效果，永远写成功，什么都不丢。
    static void writeOutNonBlocking(const std::string& line) {
        static std::atomic<bool> nb_init{false};
        if (!nb_init.exchange(true)) {
            const int fl = ::fcntl(STDOUT_FILENO, F_GETFL);
            if (fl >= 0) ::fcntl(STDOUT_FILENO, F_SETFL, fl | O_NONBLOCK);
        }
        const char* p = line.data();
        size_t left = line.size();
        while (left > 0) {
            const ssize_t n = ::write(STDOUT_FILENO, p, left);
            if (n < 0) {
                if (errno == EINTR) continue;
                Logger::getInstance().dropped_lines_.fetch_add(1, std::memory_order_relaxed);
                return;   // EAGAIN：消费端跟不上，丢行保服务
            }
            p += n;
            left -= static_cast<size_t>(n);
        }
    }

    Logger() : current_level_(LogLevel::INFO), simple_format_(false) {}
    LogLevel current_level_;
    bool simple_format_;
    std::mutex mutex_;
    std::atomic<long long> dropped_lines_{0};
};

} // namespace orbbec_service

#define LOG_DEBUG(tag, ...) orbbec_service::Logger::getInstance().log(orbbec_service::LogLevel::DEBUG, tag, __VA_ARGS__)
#define LOG_INFO(tag, ...)  orbbec_service::Logger::getInstance().log(orbbec_service::LogLevel::INFO,  tag, __VA_ARGS__)
#define LOG_WARN(tag, ...)  orbbec_service::Logger::getInstance().log(orbbec_service::LogLevel::WARN,  tag, __VA_ARGS__)
#define LOG_ERROR(tag, ...) orbbec_service::Logger::getInstance().log(orbbec_service::LogLevel::ERROR, tag, __VA_ARGS__)
