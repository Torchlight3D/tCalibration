#pragma once

#include <chrono>
#include <cstdint>

namespace tl::time {

using namespace std::chrono_literals;

using std::chrono::seconds;
using std::chrono::microseconds;
using std::chrono::nanoseconds;

using seconds_d = std::chrono::duration<double, std::chrono::seconds::period>;
static_assert(std::chrono::treat_as_floating_point_v<seconds_d::rep>,
              "seconds_d's rep required to be floating point");

// Use int64_t for nsec, double for sec
inline constexpr auto sToNs(double sec)
{
    return std::chrono::duration_cast<nanoseconds>(sec * 1s).count();
}
inline constexpr auto nsToS(int64_t ns)
{
    return std::chrono::duration_cast<seconds_d>(ns * 1ns).count();
}
inline constexpr auto sToUs(double s)
{
    return std::chrono::duration_cast<microseconds>(s * 1s).count();
}
inline constexpr auto usToS(int64_t us)
{
    return std::chrono::duration_cast<seconds_d>(us * 1us).count();
}

} // namespace tl::time
