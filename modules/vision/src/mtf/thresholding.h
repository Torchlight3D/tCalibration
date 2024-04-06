#pragma once

namespace cv {
class Mat;
}

void bradley_adaptive_threshold(const cv::Mat& cvimg, cv::Mat& img,
                                double threshold, int S);
void sauvola_adaptive_threshold(const cv::Mat& cvimg, cv::Mat& img,
                                double threshold, int S);
void invert(cv::Mat& img);
