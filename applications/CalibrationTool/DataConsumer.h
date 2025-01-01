#pragma once

#include <thread>

#include <QObject>

namespace tl {

// TODO:
// 1. Get rid of QObject
// 2. Use Channel as template variable.
// Failed to compile, because QObject can't work with template class???
class DataConsumer : public QObject
{
    Q_OBJECT

public:
    virtual ~DataConsumer() { stopAll(); }

    void start(size_t index)
    {
        if (index >= _channel) {
            return;
        }

        auto& running = _running[index];
        if (running) {
            return;
        }

        running = true;
        _threads[index] = std::thread{&DataConsumer::consume, this, index};
    }

    void startAll()
    {
        for (size_t i{0}; i < _channel; ++i) {
            start(i);
        }
    }

    void stop(size_t index)
    {
        if (index >= _channel) {
            return;
        }

        auto& running = _running[index];
        if (!running) {
            return;
        }

        running = false;

        auto& thread = _threads[index];
        if (thread.joinable()) {
            thread.join();
        }

        if (std::ranges::any_of(
                _running, [](const auto& running) { return running.load(); })) {
            return;
        }

        emit allStopped();
    }

    void stopAll()
    {
        for (size_t i{0}; i < _channel; ++i) {
            stop(i);
        }
    }

    bool stopped() const
    {
        return std::ranges::none_of(
            _running, [](const auto& running) { return running.load(); });
    }

signals:
    void allStopped();

protected:
    explicit DataConsumer(size_t channel)
        : _threads(channel), _running(channel), _channel(channel)
    {
    }

    virtual void consume(size_t channel) = 0;

protected:
    std::vector<std::thread> _threads;
    std::vector<std::atomic<bool>> _running;
    const size_t _channel;
};

} // namespace tl
