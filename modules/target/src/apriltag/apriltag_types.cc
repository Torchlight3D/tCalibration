#include "apriltag_types.h"

namespace apriltags {

namespace {
int idCounter{0};
}

Segment::Segment()
    : children(),
      point0(),
      point1(),
      theta(0.),
      length(0.),
      segmentId(++idCounter)
{
}

} // namespace apriltags
