#pragma once

#include <limits>
#include <numeric>

namespace thoht {

using TrackId = int;
inline constexpr TrackId kInvalidTrackId = std::numeric_limits<TrackId>::max();

using ViewId = int;
inline constexpr ViewId kInvalidViewId = std::numeric_limits<ViewId>::max();

using CameraId = int;
inline constexpr CameraId kInvalidCameraId =
    std::numeric_limits<CameraId>::max();

} // namespace thoht
