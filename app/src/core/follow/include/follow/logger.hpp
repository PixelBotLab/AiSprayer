// follow 的日志。格式刻意与 orbbec_camera_service 一致：
//   [INFO ] 2026-08-30 13:22:01.482 [node] 正文
// 因为 app 侧的 Python 门面用一条正则把 C++ 子进程的日志行还原成 Python log
// （camera_service.py 的 CPP_LOG_REGEX 就是这个形状）。换个格式不是"更好看"，是让
// 上层解析不到、日志被当成未知行原样吐进终端。
//
// 级别过滤在 LogLine 构造前做（宏里的 if），所以被过滤掉的行连时间戳都不取 ——
// 每帧一条 debug 行在 15 fps 下不值 30 微秒的浪费。
// 线程安全：flush 在一把全局锁里，localtime_r 而非 localtime（后者返回静态缓冲区，
// 取流线程和健康服务器线程同时打日志就会互相踩）。
// 非阻塞写出：stdout 是管道/终端时消费端可能停摆（管道写满），阻塞式 write 会把
// 打日志的线程整个冻住（实测曾把 HTTP 线程冻 90+ 秒，页面报"后端服务无响应"）。
// 所以置 O_NONBLOCK，写不进就丢行计数 —— 日志可以丢，业务线程不能冻。
#pragma once

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <unistd.h>

namespace follow {

enum class LogLevel { kDebug = 0, kInfo, kWarn, kError };

inline LogLevel& log_level_ref() {
  static LogLevel level = LogLevel::kInfo;
  return level;
}

inline void set_log_level(LogLevel l) { log_level_ref() = l; }

inline bool set_log_level(const std::string& name) {
  if (name == "debug") {
    set_log_level(LogLevel::kDebug);
  } else if (name == "info") {
    set_log_level(LogLevel::kInfo);
  } else if (name == "warn") {
    set_log_level(LogLevel::kWarn);
  } else if (name == "error") {
    set_log_level(LogLevel::kError);
  } else {
    return false;
  }
  return true;
}

inline bool log_enabled(LogLevel l) { return l >= log_level_ref(); }

class LogLine {
 public:
  LogLine(LogLevel level, const char* tag) {
    s_ << "[" << level_name(level) << "] " << stamp() << " [" << tag << "] ";
  }
  ~LogLine() {
    std::lock_guard<std::mutex> lock(mutex());
    std::string out = s_.str();
    out.push_back('\n');
    writeNonBlocking(out);
  }
  LogLine(const LogLine&) = delete;
  LogLine& operator=(const LogLine&) = delete;

  std::ostringstream& stream() { return s_; }

  static std::mutex& mutex() {
    static std::mutex m;
    return m;
  }

  // EAGAIN 丢行保线程；普通文件（重定向）上 O_NONBLOCK 无效果，永远写成功。
  static void writeNonBlocking(const std::string& line) {
    static bool nb_init = [] {
      const int fl = ::fcntl(STDOUT_FILENO, F_GETFL);
      if (fl >= 0) ::fcntl(STDOUT_FILENO, F_SETFL, fl | O_NONBLOCK);
      return true;
    }();
    (void)nb_init;
    const char* p = line.data();
    size_t left = line.size();
    while (left > 0) {
      const ssize_t n = ::write(STDOUT_FILENO, p, left);
      if (n < 0) {
        if (errno == EINTR) continue;
        return;   // 丢这一行：消费端跟不上，不能反过来冻住业务线程
      }
      p += n;
      left -= static_cast<size_t>(n);
    }
  }

 private:
  static const char* level_name(LogLevel l) {
    switch (l) {
      case LogLevel::kDebug: return "DEBUG";
      case LogLevel::kInfo: return "INFO ";
      case LogLevel::kWarn: return "WARN ";
      case LogLevel::kError: return "ERROR";
    }
    return "INFO ";
  }

  static std::string stamp() {
    const auto now = std::chrono::system_clock::now();
    const auto t = std::chrono::system_clock::to_time_t(now);
    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::tm tm_buf{};
    ::localtime_r(&t, &tm_buf);
    char date[40];
    std::snprintf(date, sizeof(date), "%04d-%02d-%02d %02d:%02d:%02d", tm_buf.tm_year + 1900,
                  tm_buf.tm_mon + 1, tm_buf.tm_mday, tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec);
    return std::string(date) + "." + ([](int v) {
      char b[8];
      std::snprintf(b, sizeof(b), "%03d", v);
      return std::string(b);
    })(static_cast<int>(ms.count()));
  }

  std::ostringstream s_;
};

// 用法：LOG_AT(LogLevel::kWarn, "node") << "帧号 " << i;
#define LOG_AT(level, tag)                                          \
  if (!::follow::log_enabled(level)) {                              \
  } else                                                            \
    ::follow::LogLine(level, tag).stream()  // NOLINT(whitespace)

}  // namespace follow
