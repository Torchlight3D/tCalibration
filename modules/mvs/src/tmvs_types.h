#pragma once

#include <limits>

namespace tl {

using TrackId = int;
inline constexpr TrackId kInvalidTrackId = std::numeric_limits<TrackId>::max();

using ViewId = int;
inline constexpr ViewId kInvalidViewId = std::numeric_limits<ViewId>::max();

using CameraId = int;
inline constexpr CameraId kInvalidCameraId =
    std::numeric_limits<CameraId>::max();

enum class PnPType
{
    KNEIP,
    SQPnP,
    DLS
};

} // namespace tl
