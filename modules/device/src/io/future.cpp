#include "future.h"

#include <cassert>
#include <deque>
#include <future>

namespace tl {

namespace {

struct InstantState : Future::State
{
    void wait() final {}
};

class StdWrapper : public Future::State
{
public:
    StdWrapper() : f(p.get_future()) {}

    void resolve() { p.set_value(); }

    void wait() final { f.wait(); }

private:
    std::promise<void> p;
    std::future<void> f;
};

} // namespace

///------- Future starts from here
Future::State::~State() = default;

Future::Future(std::shared_ptr<State> state) : state(state) {}

void Future::wait()
{
    assert(state);
    return state->wait();
}

Future Future::instantlyResolved()
{
    return Future(std::unique_ptr<Future::State>(new InstantState));
}

///------- Promise starts from here

class PromiseImplementation : public Promise
{
private:
    std::shared_ptr<StdWrapper> wrapper;

public:
    PromiseImplementation() : wrapper(new StdWrapper) {}

    void resolve() final { wrapper->resolve(); }

    Future getFuture() final { return Future(wrapper); }
};

Promise::~Promise() = default;

std::unique_ptr<Promise> Promise::create()
{
    return std::unique_ptr<Promise>(new PromiseImplementation);
}

///------- Processor starts from here

namespace {
struct BlockingQueue : Queue
{
    virtual bool waitAndProcessOne() = 0;
    virtual void processUntilDestroyed() = 0;
};

class QueueImplementation : public BlockingQueue
{
public:
    ~QueueImplementation()
    {
        std::unique_lock<std::mutex> lock(mutex);
        shouldQuit = true;
        emptyCondition.notify_all();

        subscribeCondition.wait(lock, [this] { return nSubscribed == 0; });
    }

    Future enqueue(const std::function<void()> &op) final
    {
        Task task;
        task.promise = Promise::create();
        auto future = task.promise->getFuture();
        task.func = op;

        {
            std::lock_guard<std::mutex> lock(mutex);
            tasks.emplace_back(std::move(task));
            emptyCondition.notify_one();
        }
        return future;
    }

    void waitUntilNSubscribed(int n)
    {
        // hacky
        std::unique_lock<std::mutex> lock(mutex);
        subscribeCondition.wait(lock, [this, n] { return nSubscribed == n; });
    }

    bool processOne() final { return process(false, false); }
    void processAll() final { process(true, false); }
    bool waitAndProcessOne() final { return process(false, true); }
    void processUntilDestroyed() final { process(true, true); }

private:
    bool process(bool many, bool waitForData)
    {
        bool any = false;
        std::unique_lock<std::mutex> lock(mutex);
        nSubscribed++;
        subscribeCondition.notify_all();
        do {
            if (waitForData)
                emptyCondition.wait(
                    lock, [this] { return shouldQuit || !tasks.empty(); });
            if (shouldQuit || tasks.empty()) {
                break;
            }
            auto task = std::move(tasks.front());
            tasks.pop_front();
            lock.unlock();

            task.func();
            task.promise->resolve();
            any = true;

            lock.lock();
        } while (many);

        nSubscribed--;
        subscribeCondition.notify_all();
        return any;
    }

private:
    struct Task
    {
        std::unique_ptr<Promise> promise;
        std::function<void()> func;
    };

    std::deque<Task> tasks;
    std::mutex mutex;
    std::condition_variable emptyCondition, subscribeCondition;
    bool shouldQuit = false;
    int nSubscribed = 0;
};

struct ThreadPool : Processor
{
public:
    ~ThreadPool()
    {
        queue->waitUntilNSubscribed(pool.size());
        queue.reset();
        for (auto &thread : pool) thread.join();
    }

    ThreadPool(int nThreads) : queue(new QueueImplementation)
    {
        assert(nThreads > 0);
        for (int i = 0; i < nThreads; ++i) {
            pool.emplace_back([this] { work(); });
        }
    }

    Future enqueue(const std::function<void()> &op) final
    {
        return queue->enqueue(op);
    }

private:
    std::vector<std::thread> pool;
    std::unique_ptr<QueueImplementation> queue;

    void work() { queue->processUntilDestroyed(); }
};

struct InstantProcessor : Processor
{
    Future enqueue(const std::function<void()> &op) final
    {
        op();
        return Future::instantlyResolved();
    }
};

} // namespace

Processor::~Processor() = default;

std::unique_ptr<Processor> Processor::createInstant()
{
    return std::unique_ptr<Processor>(new InstantProcessor);
}

std::unique_ptr<Processor> Processor::createThreadPool(int nThreads)
{
    return std::unique_ptr<Processor>(new ThreadPool(nThreads));
}

std::unique_ptr<Queue> Processor::createQueue()
{
    return std::unique_ptr<Queue>(new QueueImplementation);
}

} // namespace tl
