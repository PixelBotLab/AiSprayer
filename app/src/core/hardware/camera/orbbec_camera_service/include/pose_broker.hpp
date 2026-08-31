// 位姿数据面的"有新值了"通知器 —— 只做通知，不做存储、不认识 JSON。
//
// 为什么存在：Python 侧原来按 poll_hz 轮询 /follow/status 拿实时位姿。轮询对**控制面**是对的
// （点一下按钮要一个明确的成败回执），对**数据面**是错的：30fps 的位姿里绝大多数轮询拿到的都是
// 同一帧，而"最坏等一个轮询周期"直接进了跟随闭环的延迟里。改成服务推之后，闭环延迟由网络决定
// 而不是由轮询周期决定。
//
// 为什么刻意**不存 payload**：位姿只存在 FollowWorker 的快照里一份。broker 再存一份就会有两个
// 真值，"哪个更新"这个问题没有廉价答案（尤其快照是多个分支分别写的）。订阅者被唤醒后自己去
// snapshot() 拿，代价是一次结构体拷贝 —— 而这正是 HTTP 轮询路径已经在做的事。
//
// 为什么是裸唤醒原语而不是一个"流"：SSE 路由需要的是"我看到 rev=N 了，给我更新的"，
// 断线重连的客户端也只需要带 last-event-id 回来。最新值语义（不是逐帧队列）在这里是故意的：
// 跟随闭环只关心当前位姿，把中间帧排队送出只会让网络抖动变成臂的抖动。
//
// 锁顺序：publish() 只在 broker 自己的 mtx_ 里待一瞬间（自增 + 出锁后再 notify）。waitNewer()
// 返回时**不持有任何锁**（unique_lock 在函数作用域内已析构），所以调用方拿到 true 之后可以放心
// 去 follow_->snapshot() 拿 snap_mutex_ —— 两条路径永远不会同时持有对方的锁。
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>

namespace orbbec_service {

class PoseBroker {
public:
    // 一路订阅者长期占一个 httplib 线程池线程（SSE 是长连接，handler 不返回）。RK3588 上
    // 默认池只有 max(8, cores-1)=8 路，不封顶的话几个残留连接就能把**控制面**一起饿死
    // —— 而控制面卡住的表象是"点了停止没反应"，比位姿旧几帧严重得多。
    static constexpr int kMaxSubscribers = 4;
    // 心跳周期：没有新帧时也写一字节注释行。作用是让死掉的客户端能被检出（写失败 ⇒ 立刻退），
    // 顺便让中间的任何代理不至于按空闲连接掐断。200ms 是"比最快帧周期(33ms)松、比任何合理的
    // stale 判据(1s)紧"的折中。
    static constexpr int64_t kHeartbeatMs = 200;

    PoseBroker() = default;

    // 拿一个订阅名额。false = 已满，调用方**必须**回错（503），不能默默等位：等着的客户端
    // 看起来像"连上了但一帧都没收到"，那是最难查的一种失败。
    bool subscribe();
    void unsubscribe();
    int subscribers() const;
    bool hasSubscribers() const { return subscribers() > 0; }

    // 发布"快照又变了"。无订阅者时退化成一次原子自增 —— 所以生产者**每帧无条件调用**它就行，
    // 不需要判断有没有人在听；把判断留给调用方早晚会出现"忘了判断"或"判断错了永远收不到"。
    void publish();

    uint64_t revision() const { return rev_.load(); }

    // 阻塞到 revision() > seen 或超时。true = 有新值；false = 超时（调用方发心跳）。
    // 返回时不持有锁，见文件头。
    bool waitNewer(uint64_t seen, int64_t timeout_ms);

    // 服务停机：唤醒所有等待者，让它们以"没有新值"返回并尽快结束各自的连接。
    void shutdown();
    // 已停机。SSE 写入侧必须拿它当循环退出条件 —— 停机后 waitNewer() 会立刻返回 false，
    // 只看返回值的话会退化成"疯狂发心跳而永远不关连接"。
    bool stopping() const;

private:
    mutable std::mutex mtx_;
    std::condition_variable cv_;
    // rev_ 在 mtx_ 下自增，但**读**在锁外（HTTP 侧只想知道"有没有新的"，差一帧无害）。
    std::atomic<uint64_t> rev_{0};
    int subscribers_ = 0;
    bool stopping_ = false;
};

// 写完共享状态就必须让订阅者知道 —— 把这两件事绑在一个作用域对象上，而不是要求调用方
// 在每个写分支末尾手抄一行 publish()。理由很实际：漏一处的症状是"订阅方卡在旧值上，
// 而且不报任何错"，而这类 bug 在本服务里已经出现过一次（rot_frozen_ 只被写 false、从没被写
// true，于是整个静止冻结是空转）。用锁的作用域当发布的作用域，就没有"忘了发布"这种状态。
//
// 用法：把写快照那段的 `std::lock_guard<std::mutex>` 换成这一行，其余代码不动。
// **只读快照的地方不要用它**（否则每次 GET 都会产生一次推送）。
class PublishOnRelease {
public:
    PublishOnRelease(std::mutex& m, PoseBroker& broker) : lock_(m), broker_(broker) {}
    ~PublishOnRelease() {
        // 先解锁再发布：两个锁永不同时持有 ⇒ 订阅方被唤醒后去读快照时不可能和本作用域互锁。
        lock_.unlock();
        broker_.publish();
    }
    PublishOnRelease(const PublishOnRelease&) = delete;
    PublishOnRelease& operator=(const PublishOnRelease&) = delete;

private:
    std::unique_lock<std::mutex> lock_;
    PoseBroker& broker_;
};

}  // namespace orbbec_service
