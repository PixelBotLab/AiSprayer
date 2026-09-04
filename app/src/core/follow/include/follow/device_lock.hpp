// 相机是**独占**设备：两个进程同时 start() 的后果不是"谁拿到的帧少一点"，而是其中一路拿到
// 一份和彩色不对齐的深度、或者干脆打不开设备还装作在跑。所以仲裁必须在打开设备之前完成。
//
// 用 flock 而不是"先试 open，失败就当别人在用"：Orbbec SDK 的失败模式很多（UAC 节点在但
// 设备被占、profile 不匹配、固件忙），拿打开失败当互斥判据会把配置错误误报成"有人在用"。
// flock 的失败码是确定的：EWOULDBLOCK 就是有人在用，别的都是真故障。
//
// 三条实测/语义约束写在接口上，别留给调用方猜：
//   1) 锁是**内核**给的，进程死了（含 SIGKILL）自动释放 —— 不需要"清理残留锁文件"这种代码，
//      写了反而是错的（那会删掉别人正持有的锁）。
//   2) flock 按 open file description 计数，同进程内第二次 open 再 flock 不会自锁。所以这道
//      锁防不住"一个进程里开两路 pipeline"，那是调用方的 bug，得靠代码评审而不是锁。
//   3) 探端口**不能**当互斥判据：实测过 18080 在 LISTEN 而相机根本没插。服务在线 ≠ 设备空闲。
//      锁文件里写的 pid 也只用于诊断（报"谁占着"），判断权全在 flock 上。
#pragma once

#include <string>

namespace follow {

class DeviceLock {
 public:
  DeviceLock() = default;
  ~DeviceLock();
  DeviceLock(const DeviceLock&) = delete;
  DeviceLock& operator=(const DeviceLock&) = delete;

  struct Busy {
    bool held_by_other = false;
    int holder_pid = -1;    // 从锁文件里读到；读不到 = -1，不代表没人持锁
    std::string detail;     // 给人看的原文，含 errno 说明
  };

  // 非阻塞取独占锁。成功返回 true。失败时 err 里是可直接展示给人的一行原因，并尽量填 busy
  // —— 调用方要能区分"别人在用"和"这文件系统不对劲"，两者的处置完全不同。
  bool acquire(const std::string& path, std::string* err, Busy* busy = nullptr);

  // 阻塞取锁，最多等 timeout_ms。用于"等隔壁服务让出相机"这种明确意图的场景。
  bool acquire_waiting(const std::string& path, int timeout_ms, std::string* err,
                       Busy* busy = nullptr);

  void release();
  bool held() const { return fd_ >= 0; }
  const std::string& path() const { return path_; }

  // 只读锁文件内容拿持锁 pid。**不**判断锁是否真的被持有 —— 那只能靠 flock。
  static int holder_pid_of(const std::string& path);

 private:
  int fd_ = -1;
  std::string path_;
};

}  // namespace follow
