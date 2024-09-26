#include "Segment.h"


namespace AprilTags {

const float Segment::minimumLineLength = 4;

Segment::Segment()
    : children(), p0(), p1(), theta(0), length(0), segmentId(++idCounter)
{
}

float Segment::segmentLength() const { return cv::norm(p0 - p1); }

int Segment::idCounter = 0;

} // namespace AprilTags
