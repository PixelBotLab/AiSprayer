#include "thread_pool.hpp"

#include <utility>

namespace aisprayer::planner
{

ThreadPool::ThreadPool(const std::size_t worker_count)
{
  if (worker_count == 0)
  {
    throw std::invalid_argument("ThreadPool worker_count must be greater than zero");
  }

  workers_.reserve(worker_count);
  try
  {
    for (std::size_t index = 0; index < worker_count; ++index)
    {
      workers_.emplace_back(&ThreadPool::workerLoop, this);
    }
  }
  catch (...)
  {
    shutdown();
    throw;
  }
}

ThreadPool::~ThreadPool()
{
  shutdown();
}

void ThreadPool::shutdown()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_)
    {
      return;
    }

    accepting_ = false;
    stopping_ = true;
  }
  task_available_.notify_all();

  for (std::thread& worker : workers_)
  {
    if (worker.joinable())
    {
      worker.join();
    }
  }
}

std::size_t ThreadPool::workerCount() const noexcept
{
  return workers_.size();
}

void ThreadPool::workerLoop()
{
  while (true)
  {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      task_available_.wait(lock, [this] { return stopping_ || !tasks_.empty(); });

      if (tasks_.empty())
      {
        return;
      }

      task = std::move(tasks_.front());
      tasks_.pop();
    }

    task();
  }
}

}  // namespace aisprayer::planner
