#include "timer.h"

namespace tl {

Timer::Timer() { start(); }

Timer::~Timer() { [[maybe_unused]] auto _ = stop<std::chrono::milliseconds>(); }

void Timer::start() { start_ = std::chrono::system_clock::now(); }

double Timer::lapInSecond()
{
    // ...
    return 0.;
}

double Timer::elapseInSecond()
{
    return static_cast<double>(stop<std::chrono::milliseconds>()) * 1e-6;
}

double Timer::elapseInMs()
{
    return static_cast<double>(stop<std::chrono::milliseconds>()) * 1e-3;
}

} // namespace tl
