// PoseBroker 的离线契约测试。
//
// 这里每一条都对应一个"运行时看不出来"的失效模式，所以不是覆盖率装饰：
//   * 丢唤醒（notify 在自增之前发、或自增在锁外做）⇒ 客户端一直等到自己超时，
//     日志上和"网不好"一模一样；
//   * 配额不回收 ⇒ 四次断线之后推送**永久**停摆，而控制面照常；
//   * shutdown 不唤醒 ⇒ 服务停机要等满一个心跳周期才能 join 完线程。
// 时间相关的断言一律用"远大于实现耗时、远小于超时"的窗口，避免在慢机器上自己造 flaky。
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "pose_broker.hpp"

namespace orbbec_service {
namespace {

using Clock = std::chrono::steady_clock;

constexpr int64_t kWaitTimeoutMs = 500;   // 单轮等待的上限：正常情况下这一轮应该 <5ms 就返回

TEST(PoseBroker, QuietStreamTimesOutSoSenderCanHeartbeat) {
    PoseBroker b;
    const auto t0 = Clock::now();
    EXPECT_FALSE(b.waitNewer(b.revision(), 30));      // 没人发布 ⇒ 必须按时返回 false（发心跳的时机）
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t0).count();
    EXPECT_GE(elapsed, 25);
    EXPECT_LT(elapsed, 200);
}

TEST(PoseBroker, PublishWakesWaiterAndBumpsRevision) {
    PoseBroker b;
    const uint64_t before = b.revision();
    b.publish();
    EXPECT_EQ(b.revision(), before + 1);
    // seen 用的是发布**前**的号 ⇒ 有新值。
    EXPECT_TRUE(b.waitNewer(before, kWaitTimeoutMs));
    // seen 已经追上当前号 ⇒ 没有新值，别给同一个快照重复发一帧。
    EXPECT_FALSE(b.waitNewer(b.revision(), 20));
}

// 核心回归：waiter 已经进到等待里，之后才发生的 publish 必须把它叫醒。
//
// 驱动协议必须是"等待者举手 → 驱动才发"，不能按"已完成轮数"发：按轮数发会自己死锁 ——
// 轮数要等 wait 返回才前进，于是有一轮会出现"等待者在等一个还没有人来发的 publish，
// 而驱动在等那个等待者先返回"，超时后报出一条**假的**丢唤醒。（第一版就是这么错的。）
TEST(PoseBroker, NoWakeupIsLostAcrossManyRounds) {
    PoseBroker b;
    constexpr int kRounds = 200;
    std::atomic<int> misses{0};
    std::atomic<uint64_t> rounds{0};
    std::atomic<uint64_t> waiting_for{0};   // 等待者这一轮正卡在哪个号上
    std::atomic<bool> armed{false};         // 已经进到 waitNewer 里（近似，见下）

    std::thread waiter([&] {
        for (int i = 0; i < kRounds; ++i) {
            const uint64_t seen = b.revision();
            waiting_for.store(seen);
            armed.store(true);
            // 超时给到 2s：正常一轮是几毫秒，只有"永远叫不醒"才会碰到它 ⇒ 记下来的一定是
            // 真丢唤醒，而不是驱动和等待者抢时序的抖动。
            if (!b.waitNewer(seen, 2000)) ++misses;
            armed.store(false);
            rounds.fetch_add(1);
        }
    });

    while (rounds.load() < static_cast<uint64_t>(kRounds)) {
        // 只在"对方确实还在等，而且等的就是当前这个号"时发。发早了（对方还没进到 wait 里）
        // 也无害：它的谓词已经满足，waitNewer 直接返回 true。
        if (armed.load() && b.revision() <= waiting_for.load()) b.publish();
        else std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    waiter.join();
    EXPECT_EQ(rounds.load(), static_cast<uint64_t>(kRounds));
    EXPECT_EQ(misses.load(), 0) << "有唤醒被丢掉：推送会静默退化成只有心跳";
}

TEST(PoseBroker, OnePublishWakesEverySubscriber) {
    PoseBroker b;
    std::atomic<int> got{0};
    std::vector<std::thread> ts;
    for (int i = 0; i < 3; ++i) {
        const uint64_t seen = b.revision();
        ts.emplace_back([&] {
            if (b.waitNewer(seen, kWaitTimeoutMs)) ++got;
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    b.publish();                                  // 一次发布，三个等待者都要醒
    for (auto& t : ts) t.join();
    EXPECT_EQ(got.load(), 3);
}

TEST(PoseBroker, SubscriberQuotaIsEnforcedAndReclaimed) {
    PoseBroker b;
    for (int i = 0; i < PoseBroker::kMaxSubscribers; ++i) {
        ASSERT_TRUE(b.subscribe()) << "第 " << i << " 路不该被拒";
    }
    EXPECT_EQ(b.subscribers(), PoseBroker::kMaxSubscribers);
    EXPECT_FALSE(b.subscribe());                  // 满了必须拒，不能默默等位
    b.unsubscribe();
    EXPECT_TRUE(b.subscribe());                   // 断一路就要能立刻再接上
    EXPECT_EQ(b.subscribers(), PoseBroker::kMaxSubscribers);
    b.unsubscribe();
    b.unsubscribe();                              // 多解一次也不能把计数打成负数
    EXPECT_GE(b.subscribers(), 0);
}

TEST(PoseBroker, PublishWithoutSubscribersStillAdvances) {
    // 生产者**无条件** publish（不判断有没有人听）。没人听时自增必须照常发生，
    // 否则一个迟到的订阅者会拿旧 rev 当新 rev，白白错过一帧。
    PoseBroker b;
    b.publish();
    b.publish();
    EXPECT_EQ(b.revision(), 2u);
    EXPECT_FALSE(b.waitNewer(2, 20));
    EXPECT_TRUE(b.waitNewer(1, kWaitTimeoutMs));
}

TEST(PoseBroker, ShutdownReleasesBlockedWaiters) {
    PoseBroker b;
    const uint64_t seen = b.revision();
    std::atomic<bool> returned{false};
    std::thread waiter([&] {
        b.waitNewer(seen, 5000);                   // 远大于停机耗时的超时
        returned = true;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const auto t0 = Clock::now();
    b.shutdown();
    waiter.join();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t0).count();
    EXPECT_TRUE(returned.load());
    EXPECT_LT(elapsed, 1000) << "shutdown 没唤醒等待者：停机要等满一个心跳周期";
    // 停机之后不能再睡回去，否则残留连接会占着线程池线程不放。
    EXPECT_FALSE(b.waitNewer(seen, 5000));
}

}  // namespace
}  // namespace orbbec_service
