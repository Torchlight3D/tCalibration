#pragma once

#ifndef __cpp_exceptions
#define THREAD_POOL_DISABLE_EXCEPTION_HANDLING
#undef THREAD_POOL_ENABLE_WAIT_DEADLOCK_CHECK
#endif

#include <chrono>
#include <condition_variable>
#include <cstddef>
#ifdef THREAD_POOL_ENABLE_PRIORITY
#include <cstdint>
#endif
#ifndef THREAD_POOL_DISABLE_EXCEPTION_HANDLING
#include <exception>
#endif
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#ifdef THREAD_POOL_ENABLE_WAIT_DEADLOCK_CHECK
#include <stdexcept>
#endif
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace BS {

class thread_pool;

using size_t = std::size_t;

using concurrency_t =
    std::invoke_result_t<decltype(std::thread::hardware_concurrency)>;

#ifdef THREAD_POOL_ENABLE_PRIORITY

using priority_t = std::int_least16_t;

namespace pr {
constexpr priority_t highest = 32767;
constexpr priority_t high = 16383;
constexpr priority_t normal = 0;
constexpr priority_t low = -16384;
constexpr priority_t lowest = -32768;
} // namespace pr

// Macros used internally to enable or disable the priority arguments in the
// relevant functions.
#define THREAD_POOL_PRIORITY_INPUT , const priority_t priority = 0
#define THREAD_POOL_PRIORITY_OUTPUT , priority
#else
#define THREAD_POOL_PRIORITY_INPUT
#define THREAD_POOL_PRIORITY_OUTPUT
#endif

namespace this_thread {

using optional_index = std::optional<size_t>;

using optional_pool = std::optional<thread_pool*>;

class [[nodiscard]] thread_info_index
{
    friend class BS::thread_pool;

public:
    [[nodiscard]] optional_index operator()() const { return index; }

private:
    optional_index index = std::nullopt;
};

class [[nodiscard]] thread_info_pool
{
    friend class BS::thread_pool;

public:
    [[nodiscard]] optional_pool operator()() const { return pool; }

private:
    optional_pool pool = std::nullopt;
};

inline thread_local thread_info_index get_index;

inline thread_local thread_info_pool get_pool;
} // namespace this_thread

template <typename T>
class [[nodiscard]] multi_future : public std::vector<std::future<T>>
{
public:
    using std::vector<std::future<T>>::vector;

    multi_future(const multi_future&) = delete;
    multi_future& operator=(const multi_future&) = delete;
    multi_future(multi_future&&) = default;
    multi_future& operator=(multi_future&&) = default;

    [[nodiscard]] std::conditional_t<std::is_void_v<T>, void, std::vector<T>>
    get()
    {
        if constexpr (std::is_void_v<T>) {
            for (auto& future : *this) {
                future.get();
            }
            return;
        }

        std::vector<T> results;
        results.reserve(this->size());
        for (auto& future : *this) {
            results.push_back(future.get());
        }
        return results;
    }

    [[nodiscard]] size_t ready_count() const
    {
        return std::count_if(
            this->cbegin(), this->cend(), [](const auto& future) {
                return future.wait_for(std::chrono::duration<double>::zero()) ==
                       std::future_status::ready;
            });
    }

    [[nodiscard]] bool valid() const
    {
        return std::all_of(this->cbegin(), this->cend(),
                           [](const auto& future) { return future.valid(); });
    }

    void wait() const
    {
        for (const auto& future : *this) {
            future.wait();
        }
    }

    template <typename R, typename P>
    bool wait_for(const std::chrono::duration<R, P>& duration) const
    {
        const auto start_time = std::chrono::steady_clock::now();
        for (const auto& future : *this) {
            future.wait_for(duration -
                            (std::chrono::steady_clock::now() - start_time));
            if (duration < std::chrono::steady_clock::now() - start_time) {
                return false;
            }
        }
        return true;
    }

    template <typename C, typename D>
    bool wait_until(const std::chrono::time_point<C, D>& timeout_time) const
    {
        for (const auto& future : *this) {
            future.wait_until(timeout_time);
            if (timeout_time < std::chrono::steady_clock::now()) {
                return false;
            }
        }
        return true;
    }
};

class [[nodiscard]] thread_pool
{
public:
    inline thread_pool() : thread_pool(0, [] {}) {}
    inline explicit thread_pool(const concurrency_t num_threads)
        : thread_pool(num_threads, [] {})
    {
    }
    inline explicit thread_pool(const std::function<void()>& init_task)
        : thread_pool(0, init_task)
    {
    }
    thread_pool(const concurrency_t num_threads,
                const std::function<void()>& init_task)
        : thread_count(determine_thread_count(num_threads)),
          threads(std::make_unique<std::thread[]>(
              determine_thread_count(num_threads)))
    {
        create_threads(init_task);
    }

    thread_pool(const thread_pool&) = delete;
    thread_pool(thread_pool&&) = delete;
    thread_pool& operator=(const thread_pool&) = delete;
    thread_pool& operator=(thread_pool&&) = delete;

    ~thread_pool()
    {
        wait();
        destroy_threads();
    }

#ifdef THREAD_POOL_ENABLE_NATIVE_HANDLES
    [[nodiscard]] std::vector<std::thread::native_handle_type>
    get_native_handles() const
    {
        std::vector<std::thread::native_handle_type> native_handles(
            thread_count);
        for (concurrency_t i = 0; i < thread_count; ++i) {
            native_handles[i] = threads[i].native_handle();
        }
        return native_handles;
    }
#endif

    [[nodiscard]] size_t get_tasks_queued() const
    {
        const std::scoped_lock tasks_lock{tasks_mutex};
        return tasks.size();
    }

    [[nodiscard]] size_t get_tasks_running() const
    {
        const std::scoped_lock tasks_lock{tasks_mutex};
        return tasks_running;
    }

    [[nodiscard]] size_t get_tasks_total() const
    {
        const std::scoped_lock tasks_lock{tasks_mutex};
        return tasks_running + tasks.size();
    }

    [[nodiscard]] concurrency_t get_thread_count() const
    {
        return thread_count;
    }

    [[nodiscard]] std::vector<std::thread::id> get_thread_ids() const
    {
        std::vector<std::thread::id> thread_ids(thread_count);
        for (concurrency_t i = 0; i < thread_count; ++i) {
            thread_ids[i] = threads[i].get_id();
        }
        return thread_ids;
    }

#ifdef THREAD_POOL_ENABLE_PAUSE
    [[nodiscard]] bool is_paused() const
    {
        const std::scoped_lock tasks_lock{tasks_mutex};
        return paused;
    }

    void pause()
    {
        const std::scoped_lock tasks_lock{tasks_mutex};
        paused = true;
    }
#endif

    void purge()
    {
        const std::scoped_lock tasks_lock{tasks_mutex};
        while (!tasks.empty()) {
            tasks.pop();
        }
    }

    template <typename F>
    void detach_task(F&& task THREAD_POOL_PRIORITY_INPUT)
    {
        {
            const std::scoped_lock tasks_lock{tasks_mutex};
            tasks.emplace(std::forward<F>(task) THREAD_POOL_PRIORITY_OUTPUT);
        }
        task_available_cv.notify_one();
    }

    template <typename T, typename F>
    void detach_blocks(const T first_index, const T index_after_last, F&& block,
                       const size_t num_blocks = 0 THREAD_POOL_PRIORITY_INPUT)
    {
        if (index_after_last > first_index) {
            const blocks blks{first_index, index_after_last,
                              num_blocks ? num_blocks : thread_count};
            for (size_t blk = 0; blk < blks.get_num_blocks(); ++blk) {
                detach_task([block = std::forward<F>(block),
                             start = blks.start(blk), end = blks.end(blk)] {
                    block(start, end);
                } THREAD_POOL_PRIORITY_OUTPUT);
            }
        }
    }

    template <typename T, typename F>
    void detach_loop(const T first_index, const T index_after_last, F&& loop,
                     const size_t num_blocks = 0 THREAD_POOL_PRIORITY_INPUT)
    {
        if (index_after_last > first_index) {
            const blocks blks{first_index, index_after_last,
                              num_blocks ? num_blocks : thread_count};
            for (size_t blk = 0; blk < blks.get_num_blocks(); ++blk) {
                detach_task([loop = std::forward<F>(loop),
                             start = blks.start(blk), end = blks.end(blk)] {
                    for (T i = start; i < end; ++i) {
                        loop(i);
                    }
                } THREAD_POOL_PRIORITY_OUTPUT);
            }
        }
    }

    template <typename T, typename F>
    void detach_sequence(const T first_index, const T index_after_last,
                         F&& sequence THREAD_POOL_PRIORITY_INPUT)
    {
        for (T i = first_index; i < index_after_last; ++i) {
            detach_task([sequence = std::forward<F>(sequence), i] {
                sequence(i);
            } THREAD_POOL_PRIORITY_OUTPUT);
        }
    }

    void reset(const concurrency_t num_threads,
               const std::function<void()>& init_task)
    {
#ifdef THREAD_POOL_ENABLE_PAUSE
        std::unique_lock tasks_lock{tasks_mutex};
        const bool was_paused = paused;
        paused = true;
        tasks_lock.unlock();
#endif
        wait();
        destroy_threads();
        thread_count = determine_thread_count(num_threads);
        threads = std::make_unique<std::thread[]>(thread_count);
        create_threads(init_task);
#ifdef THREAD_POOL_ENABLE_PAUSE
        tasks_lock.lock();
        paused = was_paused;
#endif
    }
    inline void reset(const std::function<void()>& init_task)
    {
        reset(0, init_task);
    }
    inline void reset()
    {
        reset(0, [] {});
    }
    inline void reset(const concurrency_t num_threads)
    {
        reset(num_threads, [] {});
    }

    template <typename F, typename R = std::invoke_result_t<std::decay_t<F>>>
    [[nodiscard]] std::future<R> submit_task(
        F&& task THREAD_POOL_PRIORITY_INPUT)
    {
        const auto task_promise = std::make_shared<std::promise<R>>();
        detach_task([task = std::forward<F>(task), task_promise] {
#ifndef THREAD_POOL_DISABLE_EXCEPTION_HANDLING
            try {
#endif
                if constexpr (std::is_void_v<R>) {
                    task();
                    task_promise->set_value();
                }
                else {
                    task_promise->set_value(task());
                }
#ifndef THREAD_POOL_DISABLE_EXCEPTION_HANDLING
            }
            catch (...) {
                try {
                    task_promise->set_exception(std::current_exception());
                }
                catch (...) {
                }
            }
#endif
        } THREAD_POOL_PRIORITY_OUTPUT);
        return task_promise->get_future();
    }

    template <typename T, typename F,
              typename R = std::invoke_result_t<std::decay_t<F>, T, T>>
    [[nodiscard]] multi_future<R> submit_blocks(
        const T first_index, const T index_after_last, F&& block,
        const size_t num_blocks = 0 THREAD_POOL_PRIORITY_INPUT)
    {
        if (index_after_last > first_index) {
            const blocks blks{first_index, index_after_last,
                              num_blocks ? num_blocks : thread_count};
            multi_future<R> future;
            future.reserve(blks.get_num_blocks());
            for (size_t blk = 0; blk < blks.get_num_blocks(); ++blk) {
                future.push_back(
                    submit_task([block = std::forward<F>(block),
                                 start = blks.start(blk), end = blks.end(blk)] {
                        return block(start, end);
                    } THREAD_POOL_PRIORITY_OUTPUT));
            }
            return future;
        }
        return {};
    }

    template <typename T, typename F>
    [[nodiscard]] multi_future<void> submit_loop(
        const T first_index, const T index_after_last, F&& loop,
        const size_t num_blocks = 0 THREAD_POOL_PRIORITY_INPUT)
    {
        if (index_after_last > first_index) {
            const blocks blks(first_index, index_after_last,
                              num_blocks ? num_blocks : thread_count);
            multi_future<void> future;
            future.reserve(blks.get_num_blocks());
            for (size_t blk = 0; blk < blks.get_num_blocks(); ++blk) {
                future.push_back(
                    submit_task([loop = std::forward<F>(loop),
                                 start = blks.start(blk), end = blks.end(blk)] {
                        for (T i = start; i < end; ++i) {
                            loop(i);
                        }
                    } THREAD_POOL_PRIORITY_OUTPUT));
            }
            return future;
        }
        return {};
    }

    template <typename T, typename F,
              typename R = std::invoke_result_t<std::decay_t<F>, T>>
    [[nodiscard]] multi_future<R> submit_sequence(
        const T first_index, const T index_after_last,
        F&& sequence THREAD_POOL_PRIORITY_INPUT)
    {
        if (index_after_last > first_index) {
            multi_future<R> future;
            future.reserve(static_cast<size_t>(index_after_last - first_index));
            for (T i = first_index; i < index_after_last; ++i) {
                future.push_back(
                    submit_task([sequence = std::forward<F>(sequence), i] {
                        return sequence(i);
                    } THREAD_POOL_PRIORITY_OUTPUT));
            }
            return future;
        }
        return {};
    }

#ifdef THREAD_POOL_ENABLE_PAUSE
    void unpause()
    {
        {
            const std::scoped_lock tasks_lock{tasks_mutex};
            paused = false;
        }
        task_available_cv.notify_all();
    }
#endif

#ifdef THREAD_POOL_ENABLE_PAUSE
#define THREAD_POOL_PAUSED_OR_EMPTY (paused || tasks.empty())
#else
#define THREAD_POOL_PAUSED_OR_EMPTY tasks.empty()
#endif

    void wait()
    {
#ifdef THREAD_POOL_ENABLE_WAIT_DEADLOCK_CHECK
        if (this_thread::get_pool() == this) {
            throw wait_deadlock();
        }
#endif
        std::unique_lock tasks_lock{tasks_mutex};
        waiting = true;
        tasks_done_cv.wait(tasks_lock, [this] {
            return (tasks_running == 0) && THREAD_POOL_PAUSED_OR_EMPTY;
        });
        waiting = false;
    }

    template <typename R, typename P>
    bool wait_for(const std::chrono::duration<R, P>& duration)
    {
#ifdef THREAD_POOL_ENABLE_WAIT_DEADLOCK_CHECK
        if (this_thread::get_pool() == this) {
            throw wait_deadlock();
        }
#endif
        std::unique_lock tasks_lock{tasks_mutex};
        waiting = true;
        const bool status =
            tasks_done_cv.wait_for(tasks_lock, duration, [this] {
                return (tasks_running == 0) && THREAD_POOL_PAUSED_OR_EMPTY;
            });
        waiting = false;
        return status;
    }

    template <typename C, typename D>
    bool wait_until(const std::chrono::time_point<C, D>& timeout_time)
    {
#ifdef THREAD_POOL_ENABLE_WAIT_DEADLOCK_CHECK
        if (this_thread::get_pool() == this) {
            throw wait_deadlock();
        }
#endif
        std::unique_lock tasks_lock{tasks_mutex};
        waiting = true;
        const bool status =
            tasks_done_cv.wait_until(tasks_lock, timeout_time, [this] {
                return (tasks_running == 0) && THREAD_POOL_PAUSED_OR_EMPTY;
            });
        waiting = false;
        return status;
    }

#ifdef THREAD_POOL_ENABLE_WAIT_DEADLOCK_CHECK
    struct wait_deadlock : public std::runtime_error
    {
        wait_deadlock() : std::runtime_error("thread_pool::wait_deadlock"){};
    };
#endif

private:
    void create_threads(const std::function<void()>& init_task)
    {
        {
            const std::scoped_lock tasks_lock{tasks_mutex};
            tasks_running = thread_count;
            workers_running = true;
        }

        for (concurrency_t i = 0; i < thread_count; ++i) {
            threads[i] = std::thread(&thread_pool::worker, this, i, init_task);
        }
    }

    void destroy_threads()
    {
        {
            const std::scoped_lock tasks_lock{tasks_mutex};
            workers_running = false;
        }
        task_available_cv.notify_all();
        for (concurrency_t i = 0; i < thread_count; ++i) {
            threads[i].join();
        }
    }

    [[nodiscard]] static concurrency_t determine_thread_count(
        const concurrency_t num_threads)
    {
        if (num_threads > 0) {
            return num_threads;
        }
        if (std::thread::hardware_concurrency() > 0) {
            return std::thread::hardware_concurrency();
        }
        return 1;
    }

    void worker(const concurrency_t idx, const std::function<void()>& init_task)
    {
        this_thread::get_index.index = idx;
        this_thread::get_pool.pool = this;
        init_task();
        std::unique_lock tasks_lock(tasks_mutex);
        while (true) {
            --tasks_running;
            tasks_lock.unlock();
            if (waiting && (tasks_running == 0) &&
                THREAD_POOL_PAUSED_OR_EMPTY) {
                tasks_done_cv.notify_all();
            }
            tasks_lock.lock();
            task_available_cv.wait(tasks_lock, [this] {
                return !THREAD_POOL_PAUSED_OR_EMPTY || !workers_running;
            });
            if (!workers_running) {
                break;
            }
            {
#ifdef THREAD_POOL_ENABLE_PRIORITY
                const std::function<void()> task =
                    std::move(std::remove_const_t<pr_task&>(tasks.top()).task);
                tasks.pop();
#else
                const std::function<void()> task = std::move(tasks.front());
                tasks.pop();
#endif
                ++tasks_running;
                tasks_lock.unlock();
                task();
            }
            tasks_lock.lock();
        }
        this_thread::get_index.index = std::nullopt;
        this_thread::get_pool.pool = std::nullopt;
    }

    template <typename T>
    class [[nodiscard]] blocks
    {
    public:
        blocks(const T first_index_, const T index_after_last_,
               const size_t num_blocks_)
            : first_index(first_index_),
              index_after_last(index_after_last_),
              num_blocks(num_blocks_)
        {
            if (index_after_last > first_index) {
                const auto total_size =
                    static_cast<size_t>(index_after_last - first_index);
                if (num_blocks > total_size) {
                    num_blocks = total_size;
                }
                block_size = total_size / num_blocks;
                remainder = total_size % num_blocks;
                if (block_size == 0) {
                    block_size = 1;
                    num_blocks = (total_size > 1) ? total_size : 1;
                }
            }
            else {
                num_blocks = 0;
            }
        }

        [[nodiscard]] T start(const size_t block) const
        {
            return first_index + static_cast<T>(block * block_size) +
                   static_cast<T>(block < remainder ? block : remainder);
        }

        [[nodiscard]] T end(const size_t block) const
        {
            return (block == num_blocks - 1) ? index_after_last
                                             : start(block + 1);
        }

        [[nodiscard]] size_t get_num_blocks() const { return num_blocks; }

    private:
        size_t block_size = 0;
        size_t num_blocks = 0;
        size_t remainder = 0;
        T first_index = 0;
        T index_after_last = 0;
    };

#ifdef THREAD_POOL_ENABLE_PRIORITY
    class [[nodiscard]] pr_task
    {
        friend class thread_pool;

    public:
        explicit pr_task(const std::function<void()>& task_,
                         const priority_t priority_ = 0)
            : task(task_), priority(priority_)
        {
        }
        explicit pr_task(std::function<void()>&& task_,
                         const priority_t priority_ = 0)
            : task(std::move(task_)), priority(priority_)
        {
        }

        [[nodiscard]] friend bool operator<(const pr_task& lhs,
                                            const pr_task& rhs)
        {
            return lhs.priority < rhs.priority;
        }

    private:
        std::function<void()> task = {};
        priority_t priority = 0;
    };
#endif

#ifdef THREAD_POOL_ENABLE_PAUSE
    bool paused = false;
#endif

    std::condition_variable task_available_cv = {};
    std::condition_variable tasks_done_cv = {};

#ifdef THREAD_POOL_ENABLE_PRIORITY
    std::priority_queue<pr_task> tasks = {};
#else
    std::queue<std::function<void()>> tasks = {};
#endif

    size_t tasks_running = 0;
    mutable std::mutex tasks_mutex = {};
    concurrency_t thread_count = 0;
    std::unique_ptr<std::thread[]> threads = nullptr;
    bool waiting = false;
    bool workers_running = false;
};
} // namespace BS
