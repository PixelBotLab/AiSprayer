// GyroTimeBase 的实现。设计理由全在头注释里，这里只写"怎么做"和两处容易写错的地方。
#include "gyro_time_base.hpp"

#include <algorithm>

namespace orbbec_service {

bool GyroTimeBase::offerPair(uint64_t device_us, int64_t host_ns) {
    if (ready() || device_us == 0 || host_ns <= 0) {
        return false;  // 已冻结，或这一帧没带设备时间戳：都不许再动标定结果
    }
    // 候选钟差 = 主机接收时刻 − 设备曝光时刻。含单向传输与排队延迟（恒正），所以要取最小。
    const int64_t candidate_ns = host_ns - static_cast<int64_t>(device_us) * 1000;
    std::lock_guard<std::mutex> g(mtx_);
    if (offset_ns_.load() != 0) {
        return false;  // 双检：入队侧与帧侧是两个线程，别把"只有帧线程调这里"留给未来猜
    }
    if (collected_ == 0) {
        best_candidate_ns_ = candidate_ns;
        worst_candidate_ns_ = candidate_ns;
    } else {
        best_candidate_ns_ = std::min(best_candidate_ns_, candidate_ns);
        worst_candidate_ns_ = std::max(worst_candidate_ns_, candidate_ns);
    }
    ++collected_;
    if (collected_ < kProbePairs) {
        return false;
    }
    spread_ns_ = worst_candidate_ns_ - best_candidate_ns_;
    // 冻结值最后发布：ready() 的读者只认 offset_，看到非零时 best/spread 都已经写好了。
    offset_ns_.store(best_candidate_ns_);
    return true;
}

int64_t GyroTimeBase::toHostNs(uint64_t device_us) const {
    const int64_t offset = offset_ns_.load();
    if (offset == 0 || device_us == 0) {
        dropped_.fetch_add(1, std::memory_order_relaxed);  // 标定前的样本：宁可少几帧陀螺
        return 0;
    }
    return static_cast<int64_t>(device_us) * 1000 + offset;
}

int GyroTimeBase::pairs_used() const {
    std::lock_guard<std::mutex> g(mtx_);
    return collected_;
}

void GyroTimeBase::reset() {
    std::lock_guard<std::mutex> g(mtx_);
    offset_ns_.store(0);
    best_candidate_ns_ = 0;
    worst_candidate_ns_ = 0;
    collected_ = 0;
    spread_ns_ = 0;
    dropped_.store(0);
}

}  // namespace orbbec_service
