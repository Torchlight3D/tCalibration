#pragma once

#include <opencv2/core/mat.hpp>

namespace tl {

namespace phaseunwrap {

// Brief:
// Phase Unwrap according to Zhang's method
//
// Ref:
// "Multilevel quality-guided phase unwrapping algorithm for real-time
// three-dimensional shape reconstruction", by Song Zhang, Xiaolin Li, and
// Shing-Tung Yau (Applied Optics 2007)
cv::Mat createqualitymap(const cv::Mat phase, const cv::Mat mask);

std::vector<float> computethresholds(cv::Mat quality, const cv::Mat mask);

void unwrap(cv::Mat phase, cv::Mat quality, cv::Mat mask,
            const std::vector<float> thresholds);

} // namespace phaseunwrap

} // namespace tl
