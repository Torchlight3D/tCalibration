#pragma once

#include <opencv2/core/mat.hpp>

#include "bayer.h"

void simpleDemosaic(cv::Mat& cvimg, cv::Mat& rawimg,
                    Bayer::cfa_pattern_t cfa_pattern, Bayer::bayer_t bayer,
                    bool unbalanced_scene);
void geometricDemosaic(cv::Mat& cvimg, cv::Mat& rawimg, int target_subset = 0);
