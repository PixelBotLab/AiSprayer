#include "pose_broker.hpp"

#include <chrono>

namespace orbbec_service {

bool PoseBroker::subscribe() {
    std::lock_guard<std::mutex> lk(mtx_);
    // 满了就拒，不排队。排队的症状是客户端"连上了但一帧都没收到"，而那和"服务挂了"从外面
    // 完全一样 —— 拒掉则调用方立刻拿到一个有名有姓的原因。
    if (stopping_ || subscribers_ >= kMaxSubscribers) return false;
    ++subscribers_;
    return true;
}

void PoseBroker::unsubscribe() {
    std::lock_guard<std::mutex> lk(mtx_);
    // 配额的账必须只减不穿。重复调用（守卫被误搬、异常路径多走一次）会把计数打成负数，
    // 之后 subscribe() 的上限判定就永久失效了 —— 表现为"推送连上四次之后再也不让连"。
    if (subscribers_ > 0) --subscribers_;
}

int PoseBroker::subscribers() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return subscribers_;
}

void PoseBroker::publish() {
    {
        // 自增必须在 mtx_ 内：否则可能卡在 waitNewer() 的"检查谓词"和"进入等待"之间，
        // 那一帧的唤醒就永久丢了（订阅方只会一直等到自己超时，日志上和网不好一模一样）。
        std::lock_guard<std::mutex> lk(mtx_);
        ++rev_;
    }
    // notify 放在锁外：唤醒的回调里没有任何需要 rev_ 稳定的东西，而持锁 notify 会让被唤醒者
    // 立刻抢不到锁、白白再排一次队。生产者每帧都要过这里，不值得为它加一次上下文切换。
    cv_.notify_all();
}

bool PoseBroker::waitNewer(uint64_t seen, int64_t timeout_ms) {
    std::unique_lock<std::mutex> lk(mtx_);
    // 谓词里带 stopping_：停机时让所有等待者立刻返回，而不是等满一个心跳周期才 join 完。
    cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                 [this, seen] { return stopping_ || rev_.load() > seen; });
    return rev_.load() > seen;
}

void PoseBroker::shutdown() {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        stopping_ = true;
    }
    cv_.notify_all();
}

bool PoseBroker::stopping() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return stopping_;
}

}  // namespace orbbec_service
