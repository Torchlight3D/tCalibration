#pragma once

#include <opencv2/core/mat.hpp>

// use this when the pixel is guaranteed to be in the image boundaries
unsigned char readPixelUnsafe(const cv::Mat& image, const cv::Point& p);
// use this when at least one of the pixels to be read is outside image
// boundaries
unsigned char readPixelSafe(const cv::Mat& image, const cv::Point& p);
// use this when the read brightness value is critical, e.g. when decoding
unsigned char readPixelSafeBilinear(const cv::Mat& image, const cv::Point2d& p);
