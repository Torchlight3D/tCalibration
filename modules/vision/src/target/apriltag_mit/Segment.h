#pragma once

#include <opencv2/core/types.hpp>

namespace AprilTags {

// Represents a line fit to a set of pixels whose gradients are similiar.
class Segment
{
public:
    // Minimum number of pixels in a segment before we'll fit a line to it.
    inline static constexpr int minimumSegmentSize = 4;

    // In pixels. Calculated based on minimum plausible decoding size for Tag9
    // family.
    inline static constexpr float minimumLineLength = 4.f;

    Segment();

    const cv::Point2f& P0() const { return this->p0; }
    cv::Point2f& rP0() { return this->p0; }

    const cv::Point2f& P1() const { return this->p1; }
    cv::Point2f& rP1() { return this->p1; }

    float theta() const { return _theta; }
    void setTheta(float val) { _theta = val; }

    float length() const { return _length; }
    void setLength(float val) { _length = val; }

    float segmentLength() const;

    int id() const { return _id; }

    std::vector<Segment*> children;

private:
    cv::Point2f p0, p1;
    float _theta;  // gradient direction (points towards white)
    float _length; // length of line segment in pixels
    int _id;
    inline static int idCounter = 0;
};

} // namespace AprilTags
