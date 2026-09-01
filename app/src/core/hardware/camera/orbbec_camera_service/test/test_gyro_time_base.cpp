// GyroTimeBase 的离线回归：这个类做的事只有"两个时钟配配对、取最小、冻结"，但它错一次的
// 现象是"陀螺静默不参与跟踪"，插上相机也看不出来 —— 所以每一条契约都得在没有设备的时候钉住。
//
// 末尾另有一条跨模块用例（GyroTimeBaseSeam）：把本类的输出真的喂进 follow 的积分器。P0-1 死的
// 是两个模块中间那条缝，两边各自单测都照不到它，所以接缝需要第三条测试，不是装饰。
#include <gtest/gtest.h>

#include <vector>

#include "gyro_time_base.hpp"
#include "follow/types.hpp"  // header-only：integrate_gyro / GyroSample

namespace orbbec_service {
namespace {

constexpr int64_t kMs = 1'000'000;
constexpr int64_t kUs = 1'000;

// 一帧：设备戳 device_us、主机接收 host_ns（= 设备戳 + 真实钟差 + 这一路的延迟）
struct Pair {
    uint64_t device_us;
    int64_t host_ns;
};

int64_t hostOf(int64_t true_offset_ns, uint64_t device_us, int64_t latency_ns) {
    return static_cast<int64_t>(device_us) * kUs + true_offset_ns + latency_ns;
}

TEST(GyroTimeBase, NotReadyUntilProbePairsCollected) {
    GyroTimeBase tb;
    EXPECT_FALSE(tb.ready());
    for (int i = 0; i < GyroTimeBase::kProbePairs - 1; ++i) {
        EXPECT_FALSE(tb.offerPair(1000 + i * 66000, hostOf(5 * kMs, 1000 + i * 66000, i * kMs)));
    }
    EXPECT_FALSE(tb.ready()) << "配对没攒够就定标：延迟抖动没被平均掉";
    EXPECT_TRUE(tb.offerPair(1000 + 7 * 66000, hostOf(5 * kMs, 1000 + 7 * 66000, 7 * kMs)));
    EXPECT_TRUE(tb.ready());
    EXPECT_EQ(tb.pairs_used(), GyroTimeBase::kProbePairs);
}

// 延迟恒为正且抖动 ⇒ 候选里的最小值最接近真实钟差。取均值会把排队毛刺吃进来。
TEST(GyroTimeBase, FreezesOnLowestLatencyPair) {
    GyroTimeBase tb;
    const int64_t truth = 1'700'000'000'000LL * kMs;  // 一个 epoch 量级的钟差
    const int64_t latencies_ns[GyroTimeBase::kProbePairs] = {3 * kMs, 9 * kMs, 2 * kMs, 7 * kMs,
                                                             4 * kMs, 8 * kMs, 5 * kMs, 6 * kMs};
    bool froze = false;
    for (int i = 0; i < GyroTimeBase::kProbePairs; ++i) {
        const uint64_t dev_us = 1'000'000 + static_cast<uint64_t>(i) * 66'000;
        froze = tb.offerPair(dev_us, hostOf(truth, dev_us, latencies_ns[i])) || froze;
    }
    EXPECT_TRUE(froze) << "offerPair 必须在定标那一帧返回 true（调用方靠它只打一条日志）";
    // 候选 = 真钟差 + 该路延迟，能取到的最好结果是**最小延迟**那一对（2 ms），所以定标值天然
    // 带着这段残差。它是常量偏移 ⇒ 门的积分区间长度不变（误差只来自偏移的**变化**），
    // 因此这里的正确性标准是"取到了最小的那个"，不是"等于真值"。
    EXPECT_EQ(tb.offset_ns(), truth + 2 * kMs) << "定标值必须是延迟最小那一对";
    EXPECT_EQ(tb.spread_ns(), 7 * kMs) << "抖动上界 = 最大候选 − 最小候选";
}

// 定标之后一律不再改：一次 USB 卡顿会把候选抬高几十毫秒，跟着改就等于在时间轴上打台阶，
// 而台阶是直接进离群门误差的一阶项（误差 ≈ ω × Δ）。
TEST(GyroTimeBase, FrozenOffsetIgnoresLaterOutliers) {
    GyroTimeBase tb;
    const int64_t truth = 100 * kMs;
    for (int i = 0; i < GyroTimeBase::kProbePairs; ++i) {
        const uint64_t dev_us = 10'000 + static_cast<uint64_t>(i) * 66'000;
        tb.offerPair(dev_us, hostOf(truth, dev_us, i * kMs));
    }
    ASSERT_TRUE(tb.ready());
    const int64_t frozen = tb.offset_ns();
    for (int i = 0; i < 50; ++i) {
        const uint64_t dev_us = 1'000'000 + static_cast<uint64_t>(i) * 66'000;
        EXPECT_FALSE(tb.offerPair(dev_us, hostOf(truth, dev_us, 30'000 * kMs)));  // 大卡顿
    }
    EXPECT_EQ(tb.offset_ns(), frozen);
    EXPECT_EQ(tb.pairs_used(), GyroTimeBase::kProbePairs) << "定标后还在收集配对";
}

// 换算必须保留设备 dt：陀螺是 burst 交付（一次回调里几条，设备戳相差 5 ms，主机戳相同），
// 用到达时间打戳会把这段压成 0，积分出的旋转直接少算。
TEST(GyroTimeBase, ConversionPreservesDeviceDeltaTime) {
    GyroTimeBase tb;
    const int64_t truth = 42 * kMs;
    for (int i = 0; i < GyroTimeBase::kProbePairs; ++i) {
        const uint64_t dev_us = 500'000 + static_cast<uint64_t>(i) * 66'000;
        tb.offerPair(dev_us, hostOf(truth, dev_us, i * kMs));
    }
    ASSERT_TRUE(tb.ready());
    const int64_t a = tb.toHostNs(900'000);
    const int64_t b = tb.toHostNs(905'000);  // 同一次 burst 里的相邻两条：设备相差 5 ms
    EXPECT_EQ(b - a, 5 * kMs) << "设备 dt 被换算吃掉了";
    EXPECT_EQ(a - static_cast<int64_t>(900'000) * kUs, tb.offset_ns());
}

// 定标未完成时不能把设备域的时间戳混进队列：那会让缓冲不再单调，比缺几帧样本难查得多。
TEST(GyroTimeBase, SamplesBeforeCalibrationAreDroppedAndCounted) {
    GyroTimeBase tb;
    EXPECT_EQ(tb.toHostNs(123'456), 0) << "未定标却给出了时间戳";
    EXPECT_EQ(tb.toHostNs(123'457), 0);
    EXPECT_EQ(tb.dropped_before_ready(), 2u) << "丢弃没有计数：失效又变回静默";
}

// 停流/重连：设备可能换了时间戳原点（重启后 µs 从 0 重新开始），旧偏移必须作废重定标。
TEST(GyroTimeBase, ResetForcesRecalibrationWithNewOrigin) {
    GyroTimeBase tb;
    for (int i = 0; i < GyroTimeBase::kProbePairs; ++i) {
        const uint64_t dev_us = 10'000 + static_cast<uint64_t>(i) * 66'000;
        tb.offerPair(dev_us, hostOf(100 * kMs, dev_us, i * kMs));
    }
    ASSERT_TRUE(tb.ready());
    tb.reset();
    EXPECT_FALSE(tb.ready());
    EXPECT_EQ(tb.dropped_before_ready(), 0u);
    EXPECT_EQ(tb.toHostNs(1'000'000), 0) << "reset 后还在用旧偏移";
    // 新原点：重启后设备 µs 从很小但**非零**的值重新起算（0 的含义是"这帧没带设备戳"）
    for (int i = 0; i < GyroTimeBase::kProbePairs; ++i) {
        const uint64_t dev_us = static_cast<uint64_t>(i + 1) * 66'000;
        tb.offerPair(dev_us, hostOf(7 * kMs, dev_us, kMs));
    }
    ASSERT_TRUE(tb.ready());
    EXPECT_EQ(tb.offset_ns(), 7 * kMs + kMs);
}

// 没带设备时间戳的帧（SDK 偶尔给 0）不许当成"钟差 = 主机时刻"，那是 6 个数量级的错。
TEST(GyroTimeBase, FramesWithoutDeviceTimestampAreSkipped) {
    GyroTimeBase tb;
    for (int i = 0; i < GyroTimeBase::kProbePairs * 3; ++i) {
        EXPECT_FALSE(tb.offerPair(0, 1'700'000 * kMs + i * kMs));
    }
    EXPECT_FALSE(tb.ready());
    EXPECT_EQ(tb.pairs_used(), 0);
}

// ---------------------------------------------------------------------------------
// 跨模块接缝：GyroTimeBase 的输出直接喂 follow 的 integrate_gyro。
//
// 为什么单独要有这一条。两边的测试各测各的：GyroTimeBase 只断言"偏移取到了最小延迟那一对"，
// follow 那边只断言"给定同一域的样本能积对"。而 P0-1 恰好死在中间那条缝上 —— 换算做了，
// 但输出的仍然是另一个域，两边各自看都"正确"，合起来一个样本也用不上，且没有任何崩溃迹象。
// 这条用例把两段真接起来跑，所以它是唯一能发现"接缝错位"的离线测试。
// ---------------------------------------------------------------------------------
TEST(GyroTimeBaseSeam, CalibratedOutputIsUsableByFollowIntegrator) {
    // 实测量级：336L 上电 ~26 min 时 getTimeStampUs 末值 ≈ 1.57e9 µs；主机 epoch ≈ 1.788e18 ns。
    constexpr uint64_t kDevBaseUs = 1'569'657'460ULL;
    const int64_t kTrueOffsetNs = 1'788'000'000'000'000'000LL - static_cast<int64_t>(kDevBaseUs) * kUs;
    constexpr int64_t kFrameUs = 66'667;      // 15 fps
    constexpr int64_t kGyroUs = 5'000;        // 200 Hz
    constexpr double kW = 0.5;                // rad/s 绕 Z

    // 定标：8 对帧，每对的延迟各不相同 ⇒ 冻结值落在延迟最小那对上（带一段常量残差）。
    const int64_t latencies_ns[GyroTimeBase::kProbePairs] = {3 * kMs, 9 * kMs, 2 * kMs, 7 * kMs,
                                                             4 * kMs, 8 * kMs, 5 * kMs, 6 * kMs};
    GyroTimeBase tb;
    for (int i = 0; i < GyroTimeBase::kProbePairs; ++i) {
        const uint64_t dev_us = kDevBaseUs + i * kFrameUs;
        const bool froze = tb.offerPair(dev_us, hostOf(kTrueOffsetNs, dev_us, latencies_ns[i]));
        // 只在凑齐最后一对的那一次返回 true（调用方靠它只打一条日志），此前每次都是 false。
        EXPECT_EQ(froze, i == GyroTimeBase::kProbePairs - 1) << "第 " << i << " 对的返回值";
    }
    ASSERT_TRUE(tb.ready());

    // 帧窗口必须用 toHostNs(本帧设备戳)，不能用本帧到达时刻：后者带着本帧 USB 延迟，
    // 与冻结的最小延迟差几百 ms 时 66 ms 窗口正好框空。
    const uint64_t dev_win_us = kDevBaseUs + (GyroTimeBase::kProbePairs - 1) * kFrameUs;
    const int64_t t0 = tb.toHostNs(dev_win_us);
    const int64_t t1 = tb.toHostNs(dev_win_us + kFrameUs);
    ASSERT_GT(t0, 0);
    ASSERT_GT(t1, t0);

    // 同一段陀螺，两种写法：裸设备戳 vs 经时间基换算。
    std::vector<follow::GyroSample> raw, fixed;
    for (int64_t off = 0; off <= kFrameUs; off += kGyroUs) {
        const uint64_t dev_us = dev_win_us + off;
        const Eigen::Vector3d omega(0, 0, kW);
        raw.push_back(follow::GyroSample{static_cast<int64_t>(dev_us) * kUs, omega});
        const int64_t host_ns = tb.toHostNs(dev_us);
        ASSERT_GT(host_ns, 0);
        fixed.push_back(follow::GyroSample{host_ns, omega});
    }

    // (a) 裸设备戳：静默归零。没有崩溃、没有 NaN，输出看起来完全正常 —— 这就是 P0-1 的现场。
    const follow::GyroDelta dead = follow::integrate_gyro(raw, t0, t1);
    EXPECT_EQ(dead.samples_used, 0) << "裸设备戳竟然被主机域窗口框到了：本用例的前提已变";
    EXPECT_FALSE(dead.valid());
    EXPECT_EQ(dead.span_ns, 0);

    // (b) 经 toHostNs 换算：同一窗口立刻可用，且积分量与覆盖时长成正比（设备 dt 没被吃掉）。
    const follow::GyroDelta ok = follow::integrate_gyro(fixed, t0, t1);
    EXPECT_GT(ok.samples_used, 5) << "换算后仍积不到样本：接缝仍然错位";
    ASSERT_TRUE(ok.valid());
    // 残余缺口 = 一个采样周期（5 ms）+ 定标残差（冻结值取最小延迟那一对，而帧窗口用的是本帧
    // 自己的延迟，两者之差）。要卡的是"远小于 max_gap 门"，不是"等于零"：常量偏移不改区间长度，
    // 因而本来就不该被当成失效。
    EXPECT_LT(ok.gap_end_ns, 10 * kMs) << "缺口已接近 max_gap 门，valid() 随时会翻成 stale";
    // 补积段与真实样本对旋转的贡献等价 ⇒ 角度正比于"真实覆盖 + 补积"，不是只看真实覆盖。
    const Eigen::Matrix3d expect =
        Eigen::AngleAxisd(kW * static_cast<double>(ok.span_ns + ok.extrap_ns) * 1e-9,
                          Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    EXPECT_LT((ok.R - expect).norm(), 1e-9) << "换算后的角度与时长不成比例：dt 被动过了";
}

// 336L 现场：定标冻结的是最小 USB 延迟，后续帧到达时刻可以再晚 350~450 ms。
// follow 若拿本帧到达时刻当积分窗口，陀螺（已按最小延迟换算）一个样本也进不去；
// 必须用同一套 toHostNs(本帧设备戳)。
TEST(GyroTimeBaseSeam, ArrivalTimeMissesWhenUsbJitterExceedsFramePeriod) {
    constexpr uint64_t kDevBaseUs = 1'569'657'460ULL;
    const int64_t kTrueOffsetNs = 1'788'000'000'000'000'000LL - static_cast<int64_t>(kDevBaseUs) * kUs;
    constexpr int64_t kFrameUs = 66'667;
    constexpr int64_t kGyroUs = 5'000;
    constexpr int64_t kUsbJitterNs = 400 * kMs;

    const int64_t latencies_ns[GyroTimeBase::kProbePairs] = {3 * kMs, 9 * kMs, 2 * kMs, 7 * kMs,
                                                             4 * kMs, 8 * kMs, 5 * kMs, 6 * kMs};
    GyroTimeBase tb;
    for (int i = 0; i < GyroTimeBase::kProbePairs; ++i) {
        const uint64_t dev_us = kDevBaseUs + i * kFrameUs;
        tb.offerPair(dev_us, hostOf(kTrueOffsetNs, dev_us, latencies_ns[i]));
    }
    ASSERT_TRUE(tb.ready());

    const uint64_t t0_dev_us = kDevBaseUs + GyroTimeBase::kProbePairs * kFrameUs;
    const uint64_t t1_dev_us = t0_dev_us + kFrameUs;
    const int64_t arrival_t0 = hostOf(kTrueOffsetNs, t0_dev_us, kUsbJitterNs);
    const int64_t arrival_t1 = hostOf(kTrueOffsetNs, t1_dev_us, kUsbJitterNs);
    const int64_t aligned_t0 = tb.toHostNs(t0_dev_us);
    const int64_t aligned_t1 = tb.toHostNs(t1_dev_us);
    ASSERT_GT(aligned_t0, 0);
    ASSERT_GT(aligned_t1, aligned_t0);

    std::vector<follow::GyroSample> gyros;
    for (int64_t off = 0; off <= kFrameUs; off += kGyroUs) {
        gyros.push_back(follow::GyroSample{tb.toHostNs(t0_dev_us + off), Eigen::Vector3d(0, 0, 0.5)});
    }

    const follow::GyroDelta dead = follow::integrate_gyro(gyros, arrival_t0, arrival_t1);
    EXPECT_EQ(dead.samples_used, 0) << "到达时刻窗口在 400ms USB 抖动下居然还能框到样本";
    EXPECT_FALSE(dead.valid());

    const follow::GyroDelta ok = follow::integrate_gyro(gyros, aligned_t0, aligned_t1);
    EXPECT_GT(ok.samples_used, 5) << "同一套 toHostNs 仍积不到：接缝还是错的";
    EXPECT_TRUE(ok.valid());
}

}  // namespace
}  // namespace orbbec_service
