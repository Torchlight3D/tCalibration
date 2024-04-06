#include "undistort_stereographic.h"

Undistort_stereographic::Undistort_stereographic(const cv::Rect& r, double f,
                                                 double pitch)
    : Undistort(r), f(f), pitch(pitch)
{
}

cv::Point2d Undistort_stereographic::slow_transform_point(double col,
                                                          double row)
{
    double px = (col + offset.x - centre.x) * pitch;
    double py = (row + offset.y - centre.y) * pitch;
    double rd = sqrt(px * px + py * py); // radial distance in mm

    if (rd == 0) {
        return {centre.x - offset.x, centre.y - offset.y};
    }

    double theta = atan(rd / f);
    double ru = tan(0.5 * theta) * f * 2.0;

    px = px * ru / rd / pitch + centre.x - offset.x;
    py = py * ru / rd / pitch + centre.y - offset.y;

    return {px, py};
}

cv::Point2d Undistort_stereographic::inverse_transform_point(double col,
                                                             double row)
{
    double px = (col + offset.x - centre.x) * pitch;
    double py = (row + offset.y - centre.y) * pitch;
    double ru = sqrt(px * px + py * py); // radial distance in mm

    if (ru == 0) {
        return cv::Point2d(centre.x - offset.x, centre.y - offset.y);
    }

    double theta = atan(0.5 * ru / f);
    double rd = tan(2 * theta) * f;

    px = px * rd / ru / pitch + centre.x - offset.x;
    py = py * rd / ru / pitch + centre.y - offset.y;

    return {px, py};
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
