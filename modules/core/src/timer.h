#pragma once

#include <chrono>

namespace tl {

// TODO: This timer is too naive, try better
class Timer
{
public:
    Timer() { start(); }
    ~Timer() { [[maybe_unused]] auto _ = stop<std::chrono::milliseconds>(); }

    void start() { start_ = std::chrono::system_clock::now(); }

    template <typename Duration_t>
    typename Duration_t::rep stop()
    {
        end_ = std::chrono::system_clock::now();
        return std::chrono::duration_cast<Duration_t>(end_ - start_).count();
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start_, end_;
};

} // namespace tl
