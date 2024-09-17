#pragma once

#include <opencv2/core/mat.hpp>

namespace tl {
namespace stag {

struct Quad
{
    std::array<cv::Point2d, 4> corners;
    cv::Point3d lineInf;
    cv::Point2d center;
    cv::Mat H;
    double projectiveDistortion = 0.;

    Quad() {}
    explicit Quad(const std::array<cv::Point2d, 4> &inCorners);
    Quad(const Quad &q);
    virtual ~Quad();

    void estimateHomography();
    void calcLineAtInfinity();
    void calculateProjectiveDistortion();
};

} // namespace stag
} // namespace tl
