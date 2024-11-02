#include "Segment.h"

namespace AprilTags {

Segment::Segment()
    : children(), p0(), p1(), theta(0), length(0), segmentId(++idCounter)
{
}

float Segment::segmentLength() const { return cv::norm(p0 - p1); }

} // namespace AprilTags
