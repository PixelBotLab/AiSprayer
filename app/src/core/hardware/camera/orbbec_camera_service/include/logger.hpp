#pragma once

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

        std::lock_guard<std::mutex> lock(mutex_);

        if (simple_format_) {
            // Simplified format for parent process ingestion: [I] [Camera] message...
            char lvl_char = 'I';
            switch (level) {
                case LogLevel::DEBUG: lvl_char = 'D'; break;
                case LogLevel::INFO:  lvl_char = 'I'; break;
                case LogLevel::WARN:  lvl_char = 'W'; break;
                case LogLevel::ERROR: lvl_char = 'E'; break;
            }
            std::cout << "[" << lvl_char << "] [" << tag << "] ";
            (std::cout << ... << std::forward<Args>(args)) << std::endl;
            return;
        }

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

        std::cout << color_code << level_str << " " << ss.str() << " [" << tag << "] " << reset_code;
        (std::cout << ... << std::forward<Args>(args)) << std::endl;
    }

private:
    Logger() : current_level_(LogLevel::INFO), simple_format_(false) {}
    LogLevel current_level_;
    bool simple_format_;
    std::mutex mutex_;
};

} // namespace orbbec_service

#define LOG_DEBUG(tag, ...) orbbec_service::Logger::getInstance().log(orbbec_service::LogLevel::DEBUG, tag, __VA_ARGS__)
#define LOG_INFO(tag, ...)  orbbec_service::Logger::getInstance().log(orbbec_service::LogLevel::INFO,  tag, __VA_ARGS__)
#define LOG_WARN(tag, ...)  orbbec_service::Logger::getInstance().log(orbbec_service::LogLevel::WARN,  tag, __VA_ARGS__)
#define LOG_ERROR(tag, ...) orbbec_service::Logger::getInstance().log(orbbec_service::LogLevel::ERROR, tag, __VA_ARGS__)
