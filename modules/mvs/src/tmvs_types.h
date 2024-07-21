#pragma once

#include <algorithm>
#include <limits>
#include <ostream>

namespace tl {

using TrackId = int;
inline constexpr TrackId kInvalidTrackId = std::numeric_limits<TrackId>::max();

using ViewId = int;
inline constexpr ViewId kInvalidViewId = std::numeric_limits<ViewId>::max();

using CameraId = int;
inline constexpr CameraId kInvalidCameraId =
    std::numeric_limits<CameraId>::max();

struct ViewIdPair
{
    ViewId first, second;

    constexpr ViewIdPair(ViewId id1, ViewId id2, bool ascend = true)
        : first(ascend ? std::min(id1, id2) : id1),
          second(ascend ? std::max(id1, id2) : id2)
    {
    }

    constexpr bool operator==(const ViewIdPair& o) const
    {
        return first == o.first && second == o.second;
    }

    constexpr bool operator<(const ViewIdPair& o) const
    {
        return first < o.first || (!(first < o.first) && second < o.second);
    }

    bool isValid() const { return first != second; }
    std::pair<ViewId, ViewId> toPair() const { return {first, second}; }
};

using ViewIdTriplet = std::tuple<ViewId, ViewId, ViewId>;

enum class PnPType
{
    KNEIP,
    SQPnP,
    DLS
};

} // namespace tl

template <>
struct std::hash<tl::ViewIdPair>
{
    size_t operator()(const tl::ViewIdPair& pair) const noexcept
    {
        const hash<tl::ViewId> hasher{};
        const auto h1 = hasher(pair.first);
        const auto h2 = hasher(pair.second);
        return h1 ^ (h2 << 1);
    }
};

inline std::ostream& operator<<(std::ostream& os, const tl::ViewIdPair& pair)
{
    os << "View Pair(" << pair.first << ", " << pair.second << ")";
    return os;
}
