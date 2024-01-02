#pragma once

#include <limits>

namespace thoht {

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

} // namespace thoht
