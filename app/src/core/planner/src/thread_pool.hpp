#pragma once

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace aisprayer::planner
{

/**
 * A fixed-size thread pool that drains accepted work during shutdown.
 *
 * Task exceptions are captured by the returned future and rethrown by get().
 * Callers retain submission order by storing futures in their original vector
 * order and consuming that vector sequentially.
 */
class ThreadPool
{
public:
  explicit ThreadPool(std::size_t worker_count);
  ~ThreadPool();

  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;
  ThreadPool(ThreadPool&&) = delete;
  ThreadPool& operator=(ThreadPool&&) = delete;

  template <typename Function, typename... Args>
  auto submit(Function&& function, Args&&... args)
      -> std::future<std::invoke_result_t<Function, Args...>>
  {
    using Result = std::invoke_result_t<Function, Args...>;

    auto task = std::make_shared<std::packaged_task<Result()>>(
        std::bind(std::forward<Function>(function), std::forward<Args>(args)...));
    std::future<Result> result = task->get_future();

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!accepting_)
      {
        throw std::runtime_error("cannot submit a task to a stopped ThreadPool");
      }

      tasks_.emplace([task] { (*task)(); });
    }
    task_available_.notify_one();
    return result;
  }

  /** Stop accepting work, drain accepted tasks, and join all workers. */
  void shutdown();

  [[nodiscard]] std::size_t workerCount() const noexcept;

private:
  void workerLoop();

  mutable std::mutex mutex_;
  std::condition_variable task_available_;
  std::queue<std::function<void()>> tasks_;
  std::vector<std::thread> workers_;
  bool accepting_{ true };
  bool stopping_{ false };
};

}  // namespace aisprayer::planner
