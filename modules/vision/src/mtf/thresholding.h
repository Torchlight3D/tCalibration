#pragma once

namespace cv {
class Mat;
}

// Ref:
// D. Bradley, G. Roth. ACM Journal of Graphics Tools. 2007. Vol 12, No. 2:
// 13-21.
void bradley_adaptive_threshold(const cv::Mat& cvimg, cv::Mat& img,
                                double threshold, int S);

// Reference:
// Efficient Implementation of Local Adaptive Thresholding Techniques Using
// Integral Images, F. Shafait, D. Keysers, T.M. Breuel, ...
void sauvola_adaptive_threshold(const cv::Mat& cvimg, cv::Mat& img,
                                double threshold, int S);

void invert(cv::Mat& img);
