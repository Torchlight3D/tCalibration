#pragma once

#include <limits>

namespace tl {

using CornerId = int;
inline constexpr auto kInvalidCornerId = std::numeric_limits<CornerId>::max();

enum class CalibBoardType
{
    ChArUco,
    Chessboard,
    CircleGridBoard,
    KalibrAprilTagBoard,
};

} // namespace tl
