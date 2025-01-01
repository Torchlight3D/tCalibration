#pragma once

namespace tl {
namespace zeromq {

inline constexpr auto kMonoDataPort{5555};
inline constexpr auto kStereoDataPort{5555};
inline constexpr auto kMotionDataPort{5556};
[[deprecated]] inline constexpr auto kSurveillanceImageDataPort{5557};
inline constexpr auto kRectifiedMonoDataPort{5559};
inline constexpr auto kRectifiedStereoDataPort{5559};
inline constexpr auto kRealtimeMotionDataPort{5560};

} // namespace zeromq
} // namespace tl
