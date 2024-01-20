#pragma once

#include <limits>

namespace tl {

using CornerId = int;
inline constexpr CornerId kInvalidCornerId =
    std::numeric_limits<CornerId>::max();

enum class CalibBoardType
{
    CheckerBoard,
    Circles,
    AprilTag,
    KalibrAprilTag,
    ChArUco,
    Caltag
};

} // namespace tl
