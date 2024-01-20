#pragma once

#include <chrono>
#include <condition_variable>
#include <exception>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace tl {

// Convenience type for std::thread::hardware_concurrency() return type.
// Should evaluate to unsigned int under Linux
using concurrency_t =
    std::invoke_result_t<decltype(std::thread::hardware_concurrency)>;

// A wrapper for future list
template <typename T>
class [[nodiscard]] FutureList
{
public:
    explicit FutureList(size_t count = 0) : futures_(count) {}

    [[nodiscard]] std::conditional_t<std::is_void_v<T>, void, std::vector<T>>
    get()
    {
        if constexpr (std::is_void_v<T>) {
            for (auto& future : futures_) {
                future.get();
            }
            return;
        }

        std::vector<T> results(futures_.size());
        for (size_t i{0}; i < futures_.size(); ++i) {
            results[i] = futures_[i].get();
        }
        return results;
    }

    [[nodiscard]] std::future<T>& operator[](size_t i) { return futures_[i]; }

    void push_back(std::future<T> future)
    {
        futures_.push_back(std::move(future));
    }

    [[nodiscard]] size_t size() const { return futures_.size(); }

    void wait() const
    {
        for (const auto& future : futures_) {
            future.wait();
        }
    }

private:
    std::vector<std::future<T>> futures_;
};

// A helper class to divide a range into blocks.
// T1 and T2 should be signed or unsigned integer type. We use two template
// parameters to adapt mixup input.
template <typename T1, typename T2, typename T = std::common_type_t<T1, T2>>
class [[nodiscard]] Blocks
{
public:
    Blocks(T1 begin, T2 end, size_t blockCount)
        : begin_(static_cast<T>(begin)),
          end_(static_cast<T>(end)),
          block_count_(blockCount)
    {
        static_assert(std::is_integral_v<T>);

        if (end_ < begin_) {
            std::swap(end_, begin_);
        }

        size_ = static_cast<size_t>(end_ - begin_);
        block_size_ = static_cast<size_t>(size_ / block_count_);
        if (block_size_ == 0) {
            block_size_ = 1;
            block_count_ = (size_ > 1) ? size_ : 1;
        }
    }

    [[nodiscard]] T start(size_t i) const
    {
        return static_cast<T>(i * block_size_) + begin_;
    }

    [[nodiscard]] T end(size_t i) const
    {
        return (i == block_count_ - 1)
                   ? end_
                   : (static_cast<T>((i + 1) * block_size_) + begin_);
    }

    [[nodiscard]] size_t blockCount() const { return block_count_; }
    [[nodiscard]] size_t size() const { return size_; }

private:
    T begin_{0}, end_{0};
    size_t block_size_ = 0;
    size_t block_count_ = 0;
    size_t size_ = 0;
};

class [[nodiscard]] ThreadPool
{
public:
    // ThreadPool uses total number of hardware threads available by default,
    // which is usually determined by the number of cores in the CPU. If a core
    // is hyperthreaded, it will count as two threads.
    explicit ThreadPool(concurrency_t threadCount = 0)
        : thread_count_(idealThreadCount(threadCount)),
          threads_(std::make_unique<std::thread[]>(thread_count_))
    {
        createThreads();
    }

    ~ThreadPool()
    {
        waitForDone();
        destroyThreads();
    }

    [[nodiscard]] size_t waitingTaskCount() const
    {
        const std::scoped_lock locker(tasks_mutex_);
        return waiting_tasks_.size();
    }

    [[nodiscard]] size_t runningTaskCount() const
    {
        const std::scoped_lock locker(tasks_mutex_);
        return running_task_count_;
    }

    [[nodiscard]] size_t totalTaskCount() const
    {
        const std::scoped_lock locker(tasks_mutex_);
        return running_task_count_ + waiting_tasks_.size();
    }

    [[nodiscard]] concurrency_t threadCount() const { return thread_count_; }

    // Parallelize a loop by automatically splitting it into blocks and
    // submitting each block separately to the queue. Returns a FutureList that
    // contains the futures for all of the blocks.
    // NOTE:
    // 1. The loop will iterate in range of [begin, end)
    // 2. The loop function should take EXACTLY TWO arguments, which are begin
    // and end.
    template <
        typename Func, typename T1, typename T2,
        typename T = std::common_type_t<T1, T2>,
        typename Return_t = std::invoke_result_t<std::decay_t<Func>, T, T>>
    [[nodiscard]] FutureList<Return_t> parallelize_loop(T1 begin, T2 end,
                                                        Func&& loop,
                                                        size_t blockCount = 0)
    {
        Blocks blocks{begin, end, blockCount ? blockCount : thread_count_};
        if (blocks.size() > 0) {
            FutureList<Return_t> futures(blocks.blockCount());
            for (size_t i{0}; i < blocks.blockCount(); ++i) {
                futures[i] = submit(std::forward<Func>(loop), blocks.start(i),
                                    blocks.end(i));
            }
            return futures;
        }

        return FutureList<Return_t>();
    }

    template <
        typename Func, typename T,
        typename Return_t = std::invoke_result_t<std::decay_t<Func>, T, T>>
    [[nodiscard]] inline auto parallelize_loop(T end, Func&& loop,
                                               size_t blockCount = 0)
    {
        return parallelize_loop(0, end, std::forward<Func>(loop), blockCount);
    }

    // Worker will stop receiving new tasks, while if there are running tasks
    // exist, it will keep running until they finish.
    void pause()
    {
        const std::scoped_lock locker(tasks_mutex_);
        paused_ = true;
    }

    [[nodiscard]] bool is_paused() const
    {
        const std::scoped_lock locker(tasks_mutex_);
        return paused_;
    }

    void resume()
    {
        const std::scoped_lock locker(tasks_mutex_);
        paused_ = false;
    }

    // Remove all the waiting tasks. Tasks that are currently running will not
    // be affected.
    void purge()
    {
        const std::scoped_lock tasks_lock(tasks_mutex_);
        while (!waiting_tasks_.empty()) {
            waiting_tasks_.pop();
        }
    }

    // Parallelize a loop by automatically splitting it into blocks and
    // submitting each block separately to the queue. Different from
    // parallelize_loop(), this method does not return a FutureList, so the user
    // must use waitForDone() or some other alternatives to ensure that the loop
    // finishes executing, otherwise bad things will happen.
    // NOTE:
    // 1. The loop will iterate in range of [begin, end)
    // 2. The loop function should take EXACTLY TWO arguments, which are begin
    // and end.
    template <typename Func, typename T1, typename T2,
              typename T = std::common_type_t<T1, T2>>
    void push_loop(T1 begin, T2 end, Func&& loop, size_t blockCount = 0)
    {
        Blocks blocks{begin, end, blockCount ? blockCount : thread_count_};
        if (blocks.size() > 0) {
            for (size_t i{0}; i < blocks.blockCount(); ++i) {
                push_task(std::forward<Func>(loop), blocks.start(i),
                          blocks.end(i));
            }
        }
    }

    template <typename Func, typename T>
    inline void push_loop(T end, Func&& loop, size_t blockCount = 0)
    {
        push_loop(0, end, std::forward<Func>(loop), blockCount);
    }

    // Push a function with zero or more arguments into the task queue. This
    // method doesn't return a future, so the user must use waitForDone() or
    // some other alternatives to ensure that the tasks finish executing,
    // otherwise bad things will happen.
    template <typename Func, typename... Args>
    void push_task(Func&& task, Args&&... args)
    {
        {
            const std::scoped_lock tasks_lock(tasks_mutex_);
            waiting_tasks_.push(std::bind(std::forward<Func>(task),
                                          std::forward<Args>(args)...));
        }
        task_available_.notify_one();
    }

    // Reset the number of threads in the pool.
    // Waits for all currently running tasks to be completed, then destroys all
    // threads in the pool, and creates a new thread pool with the new number of
    // threads. Any tasks that were waiting in the queue before the pool was
    // reset will be executed by the NEW threads. If the pool was paused before
    // resetting it, the new pool will be paused as well.
    void reset(concurrency_t threadCount = 0)
    {
        std::unique_lock locker(tasks_mutex_);
        const bool was_paused = paused_;
        paused_ = true;
        locker.unlock();

        waitForDone();
        destroyThreads();
        thread_count_ = idealThreadCount(threadCount);
        threads_ = std::make_unique<std::thread[]>(thread_count_);
        paused_ = was_paused;
        createThreads();
    }

    // Submit a function with zero or more arguments into the task queue.
    // Return the future of the return value (if not, then void) of input
    // function, which can be used to wait until the task finishes.
    template <typename Func, typename... Args,
              typename Return_t = std::invoke_result_t<std::decay_t<Func>,
                                                       std::decay_t<Args>...>>
    [[nodiscard]] std::future<Return_t> submit(Func&& func, Args&&... args)
    {
        auto promise = std::make_shared<std::promise<Return_t>>();
        push_task([task = std::bind(std::forward<Func>(func),
                                    std::forward<Args>(args)...),
                   promise] {
            try {
                if constexpr (std::is_void_v<Return_t>) {
                    std::invoke(task);
                    promise->set_value();
                }
                else {
                    promise->set_value(std::invoke(task));
                }
            }
            catch (...) {
                try {
                    promise->set_exception(std::current_exception());
                }
                catch (...) {
                }
            }
        });
        return promise->get_future();
    }

    // Wait for tasks to be completed. Normally, this method waits for all tasks
    // (both running and waiting tasks). However, if the pool is paused, this
    // function only waits for the currently running tasks, otherwise it would
    // wait forever.
    // NOTE: To wait for just one specific task, use submit() instead, and call
    // the wait() member function of the generated future.
    void waitForDone()
    {
        std::unique_lock locker(tasks_mutex_);
        waiting_ = true;
        tasks_done_.wait(locker, [this] {
            return !running_task_count_ && (paused_ || waiting_tasks_.empty());
        });
        waiting_ = false;
    }

    // Wait for tasks to be completed, but stop waiting after the specified
    // duration has passed. Return true if all tasks finished running, false if
    // the duration expired but some tasks are still running.
    template <typename Rep_t, typename Period_t>
    bool waitForDoneFor(const std::chrono::duration<Rep_t, Period_t>& duration)
    {
        std::unique_lock locker(tasks_mutex_);
        waiting_ = true;
        const bool status = tasks_done_.wait_for(locker, duration, [this] {
            return !running_task_count_ && (paused_ || waiting_tasks_.empty());
        });
        waiting_ = false;
        return status;
    }

    // Wait for tasks to be completed, but stop waiting after the specified time
    // point has been reached. Return true if all tasks finished running, false
    // if the time point was reached but some tasks are still running.
    template <typename Clock_t, typename Duration_t>
    bool waitForDoneUntil(
        const std::chrono::time_point<Clock_t, Duration_t>& timeout)
    {
        std::unique_lock locker(tasks_mutex_);
        waiting_ = true;
        const bool status = tasks_done_.wait_until(locker, timeout, [this] {
            return !running_task_count_ && (paused_ || waiting_tasks_.empty());
        });
        waiting_ = false;
        return status;
    }

private:
    void createThreads()
    {
        {
            const std::scoped_lock locker(tasks_mutex_);
            workers_running_ = true;
        }

        for (concurrency_t i = 0; i < thread_count_; ++i) {
            threads_[i] = std::thread(&ThreadPool::worker, this);
        }
    }

    void destroyThreads()
    {
        {
            const std::scoped_lock locker(tasks_mutex_);
            workers_running_ = false;
        }

        task_available_.notify_all();
        for (concurrency_t i = 0; i < thread_count_; ++i) {
            threads_[i].join();
        }
    }

    [[nodiscard]] static concurrency_t idealThreadCount(concurrency_t count)
    {
        if (count > 0) {
            return count;
        }

        if (std::thread::hardware_concurrency() > 0) {
            return std::thread::hardware_concurrency();
        }

        return 1;
    }

    // The worker function to be assigned to each thread in the pool. Waits
    // until it is notified by push_task() that a task is available, and then
    // retrieves the task from the queue and executes it. Once the task
    // finishes, the worker notifies waitForDone() in case it is waiting.
    void worker()
    {
        std::function<void()> task;
        while (true) {
            std::unique_lock locker(tasks_mutex_);
            task_available_.wait(locker, [this] {
                return !waiting_tasks_.empty() || !workers_running_;
            });

            if (!workers_running_) {
                break;
            }
            if (paused_) {
                continue;
            }

            task = std::move(waiting_tasks_.front());
            waiting_tasks_.pop();
            ++running_task_count_;
            locker.unlock();
            task();
            locker.lock();
            --running_task_count_;
            if (waiting_ && !running_task_count_ &&
                (paused_ || waiting_tasks_.empty())) {
                tasks_done_.notify_all();
            }
        }
    }

private:
    std::condition_variable task_available_ = {};
    std::condition_variable tasks_done_ = {};
    std::queue<std::function<void()>> waiting_tasks_ = {};
    concurrency_t thread_count_ = 0;
    std::unique_ptr<std::thread[]> threads_ = nullptr;
    mutable std::mutex tasks_mutex_ = {};
    size_t running_task_count_ = 0;
    bool paused_ = false;
    bool waiting_ = false;
    bool workers_running_ = false;
};

// A helper class to synchronize printing to an output stream by different
// threads.
class [[nodiscard]] SyncedStream
{
public:
    explicit SyncedStream(std::ostream& stream = std::cout) : oss_(stream) {}

    // Thread safe print
    template <typename... T>
    void print(T&&... items)
    {
        const std::scoped_lock lock{mtx_};
        (oss_ << ... << std::forward<T>(items));
    }

    template <typename... T>
    inline void println(T&&... items)
    {
        print(std::forward<T>(items)..., '\n');
    }

    // endl to use with SyncedStream
    inline static std::ostream& (&endl)(std::ostream&) =
        static_cast<std::ostream& (&)(std::ostream&)>(std::endl);

    // flush to use with SyncedStream
    inline static std::ostream& (&flush)(std::ostream&) =
        static_cast<std::ostream& (&)(std::ostream&)>(std::flush);

private:
    std::ostream& oss_;
    mutable std::mutex mtx_ = {};
};

} // namespace tl
