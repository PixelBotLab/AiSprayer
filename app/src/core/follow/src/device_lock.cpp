#include "follow/device_lock.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>

namespace follow {
namespace {

constexpr int kRetryStepMs = 50;

// O_CLOEXEC（acquire 里的开法）是安全条件不是优化：持锁进程 exec 出子进程时会继承 fd 从而
// **继续**持有这把锁，而父进程一退出，所有后来者都以为锁已经放了。
bool parent_dir_exists(const std::string& path) {
  const auto slash = path.find_last_of('/');
  if (slash == std::string::npos || slash == 0) {
    return true;
  }
  struct stat st {};
  return ::stat(path.substr(0, slash).c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

void write_holder(int fd) {
  if (::ftruncate(fd, 0) != 0) {
    return;
  }
  char buf[48];
  const int n = std::snprintf(buf, sizeof(buf), "%d\n", static_cast<int>(::getpid()));
  if (n > 0 && ::write(fd, buf, static_cast<size_t>(n)) < 0) {
    // 写不进去只影响"谁占着"这一条诊断信息，锁本身已经拿到了
  }
}

int read_pid(const std::string& path) {
  int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0) {
    return -1;
  }
  char buf[32] = {0};
  const ssize_t n = ::read(fd, buf, sizeof(buf) - 1);
  ::close(fd);
  if (n <= 0) {
    return -1;
  }
  long v = 0;
  if (std::sscanf(buf, "%ld", &v) != 1 || v <= 0 || v > 4194304) {
    return -1;
  }
  return static_cast<int>(v);
}

}  // namespace

DeviceLock::~DeviceLock() { release(); }

bool DeviceLock::acquire(const std::string& path, std::string* err, Busy* busy) {
  release();
  if (err) {
    err->clear();
  }
  const int fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0644);
  if (fd < 0) {
    if (err) {
      *err = "打不开锁文件 " + path + ": " + std::strerror(errno) +
             (parent_dir_exists(path) ? "" : "（父目录不存在）");
    }
    return false;
  }
  if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
    const int saved = errno;
    ::close(fd);
    if (saved == EWOULDBLOCK || saved == EAGAIN) {
      if (busy) {
        busy->held_by_other = true;
        busy->holder_pid = read_pid(path);
        busy->detail = "相机锁被进程 " + std::to_string(busy->holder_pid) + " 持有";
      }
      if (err) {
        *err = busy && busy->holder_pid > 0 ? busy->detail + " —— 先停掉它再启 follow"
                                            : "相机锁已被占用（" + path + "）—— 先停掉持锁进程";
      }
    } else {
      if (err) {
        *err = "flock(" + path + ") 失败: " + std::strerror(saved);
      }
    }
    return false;
  }
  write_holder(fd);
  fd_ = fd;
  path_ = path;
  return true;
}

bool DeviceLock::acquire_waiting(const std::string& path, int timeout_ms, std::string* err,
                                 Busy* busy) {
  const auto t0 = std::chrono::steady_clock::now();
  for (;;) {
    if (acquire(path, err, busy)) {
      return true;
    }
    if (!busy || !busy->held_by_other) {
      return false;  // 不是"有人在用"，等下去也不会变
    }
    const int elapsed = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0)
            .count());
    if (elapsed >= timeout_ms) {
      if (err) {
        *err = "等待相机锁超时 " + std::to_string(timeout_ms) + " ms，" + *err;
      }
      return false;
    }
    ::usleep(static_cast<useconds_t>(kRetryStepMs) * 1000);
  }
}

void DeviceLock::release() {
  if (fd_ >= 0) {
    ::flock(fd_, LOCK_UN);
    ::close(fd_);
    fd_ = -1;
  }
  path_.clear();
}

int DeviceLock::holder_pid_of(const std::string& path) { return read_pid(path); }

}  // namespace follow
