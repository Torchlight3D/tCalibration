#pragma once

#include <opencv2/core/types.hpp>

namespace AprilTags {

//! Represents a line fit to a set of pixels whose gradients are similiar.
class Segment
{
public:
    Segment();

    //!< Minimum number of pixels in a segment before we'll fit a line to
    //!< it.
    inline static constexpr int minimumSegmentSize = 4;

    //!< In pixels. Calculated based on minimum plausible
    //!< decoding size for Tag9 family.
    inline static constexpr float minimumLineLength = 4.f;

    const cv::Point2f& P0() const { return this->p0; }
    cv::Point2f& rP0() { return this->p0; }

    const cv::Point2f& P1() const { return this->p1; }
    cv::Point2f& rP1() { return this->p1; }

    float getTheta() const { return theta; }
    void setTheta(float newValue) { theta = newValue; }

    float getLength() const { return length; }
    void setLength(float newValue) { length = newValue; }

    //! Returns the length of the Segment.
    float segmentLength() const;

    //! ID of Segment.
    int getId() const { return segmentId; }

    std::vector<Segment*> children;

private:
    cv::Point2f p0, p1;
    float theta;  // gradient direction (points towards white)
    float length; // length of line segment in pixels
    int segmentId;
    inline static int idCounter = 0;
};

} // namespace AprilTags
