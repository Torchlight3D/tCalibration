#pragma once

#include <cstdint>

namespace tl::time {

// Use int64_t for nsec, double for sec
inline constexpr int64_t sToNs(double sec) { return sec * 1e9; }
inline constexpr double nsToS(int64_t ns) { return ns * 1e-9; }
inline constexpr double sToUs(double s) { return s * 1e6; }
inline constexpr double usToS(double us) { return us * 1e-6; }

} // namespace tl::time
