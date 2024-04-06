#pragma once

#include <vector>

#include <opencv2/core/types.hpp>

#include "ordered_point.h"

double loess_core(std::vector<Ordered_point>& ordered, size_t start_idx,
                  size_t end_idx, double mid, cv::Point2d& sol);

// FIXME: Where is the declaration
int bin_fit(std::vector<Ordered_point>& ordered, double* fft_in_buffer,
            int fft_size, double lower, double upper, std::vector<double>& esf,
            bool allow_peak_shift = false);
