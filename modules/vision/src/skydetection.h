#pragma once

#include <opencv2/core/mat.hpp>

namespace tl {

namespace SkyAreaDetector {

struct Params
{
    double f_thres_sky_max = 600;
    double f_thres_sky_min = 5;
    double f_thres_sky_search_step = 5;
};

void detect(cv::InputArray image, cv::OutputArray skyMask,
            const Params& parameters, cv::OutputArray viz = cv::noArray());

} // namespace SkyAreaDetector

} // namespace tl
