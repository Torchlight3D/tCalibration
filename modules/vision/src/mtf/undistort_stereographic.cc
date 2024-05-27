#include "undistort_stereographic.h"

Undistort_stereographic::Undistort_stereographic(const cv::Rect& r, double f,
                                                 double pitch)
    : Undistort(r), f(f), pitch(pitch)
{
}

cv::Point2d Undistort_stereographic::slow_transform_point(
    const cv::Point2d& point) const
{
    const auto pt = (point + offset - centre) * pitch;
    // radial distance in mm
    const auto rd = cv::norm(pt);

    if (rd == 0.) {
        return centre - offset;
    }

    double theta = std::atan(rd / f);
    double ru = std::tan(0.5 * theta) * f * 2.0;

    return pt * (ru / rd / pitch) + centre - offset;
}

cv::Point2d Undistort_stereographic::inverse_transform_point(
    const cv::Point2d& point) const
{
    const auto pt = (point + offset - centre) * pitch;
    // radial distance in mm
    double ru = cv::norm(pt);

    if (ru == 0.) {
        return centre - offset;
    }

    double theta = std::atan(0.5 * ru / f);
    double rd = std::tan(2 * theta) * f;

    return pt * (rd / ru / pitch) + centre - offset;
}

// note: second parameter is the raw Bayer image, which must also be padded out
cv::Mat Undistort_stereographic::unmap(const cv::Mat& in_src, cv::Mat& rawimg)
{
    int pad_left = 0;
    int pad_top = 0;
    estimate_padding(in_src, pad_left, pad_top);

    // TODO: perform some sanity checking ...
    pad_left = std::min(8000, pad_left);
    pad_top = std::min(8000, pad_top);

    return unmap_base(in_src, rawimg, pad_left, pad_top);
}
