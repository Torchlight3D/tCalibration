#include "Segment.h"

namespace AprilTags {

Segment::Segment()
    : children(), p0(), p1(), _theta(0), _length(0), _id(++idCounter)
{
}

float Segment::segmentLength() const { return cv::norm(p0 - p1); }

} // namespace AprilTags
