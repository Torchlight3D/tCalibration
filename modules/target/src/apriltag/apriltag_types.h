#pragma once

#include <iostream>
#include <vector>
#include <opencv2/core/types.hpp>

#include <tMath/MathBase>

namespace apriltags {

inline std::ostream& operator<<(std::ostream& os, const cv::Point2f& pt)
{
    os << pt.x << ", " << pt.y;
    return os;
}

// TODO: maybe use template?
struct WeightedPointF
{
    cv::Point2f point;
    float weight;

    WeightedPointF(float x, float y, float weight) : point(x, y), weight(weight)
    {
    }
};

inline std::ostream& operator<<(std::ostream& os, const WeightedPointF& wpt)
{
    os << wpt.point << ", "
       << "weight: " << wpt.weight;
    return os;
}

//! Represents a line fit to a set of pixels whose gradients are similiar.
class Segment
{
public:
    Segment();

    static constexpr int kMinSegmentSize{4};   //!< In pixels.
    static constexpr float kMinLineLength{4.}; //!< In pixels.

    int getId() const { return segmentId; }

public:
    std::vector<Segment*> children;
    cv::Point2f point0, point1;
    float theta;  // gradient direction (points towards white)
    float length; // length of line segment in pixels

private:
    int segmentId;
};

inline std::ostream& operator<<(std::ostream& os, const Segment& seg)
{
    os << "(" << seg.point0 << "), "
       << "(" << seg.point1 << ")";
    return os;
}

} // namespace apriltags
