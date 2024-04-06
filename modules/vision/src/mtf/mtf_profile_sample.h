#pragma once

#include <opencv2/core/types.hpp>

class Mtf_profile_sample
{
public:
    Mtf_profile_sample(const cv::Point2d& p, double mtf, double angle,
                       double quality = 1.0)
        : p(p), mtf(mtf), angle(angle), quality(quality)
    {
    }

    cv::Point2d p;
    double mtf;
    double angle;
    double quality;
};
