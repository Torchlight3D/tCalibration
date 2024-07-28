#pragma once

#include <opencv2/core/mat.hpp>

namespace Srgb_render {

cv::Mat linear_to_sRGB(const cv::Mat& img);
unsigned char reverse_gamma(double x);

}; // namespace Srgb_render
