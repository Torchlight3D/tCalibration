#include "qualitymeasurement.h"

#include <algorithm>
#include <cmath>

namespace tl {

double InlierSupport::ComputeCost(const std::vector<double>& residuals,
                                  std::vector<size_t>* inliers)
{
    inliers->reserve(residuals.size());
    for (size_t i{0}; i < residuals.size(); i++) {
        if (residuals[i] < _threshold) {
            inliers->emplace_back(i);
        }
    }

    return static_cast<double>(residuals.size() - inliers->size());
}

double MLEQualityMeasurement::ComputeCost(const std::vector<double>& residuals,
                                          std::vector<size_t>* inliers)
{
    inliers->reserve(residuals.size());

    auto cost{0.};
    for (size_t i{0}; i < residuals.size(); i++) {
        if (residuals[i] < _threshold) {
            cost += residuals[i];
            inliers->emplace_back(i);
        }
        else {
            cost += _threshold;
        }
    }

    return cost;
}

LmedQualityMeasurement::LmedQualityMeasurement(size_t minSampleSize)
    : QualityMeasurement(), _minSampleSize(minSampleSize)
{
}

double LmedQualityMeasurement::ComputeCost(const std::vector<double>& residuals,
                                           std::vector<size_t>* inliers)
{
    // TODO: Check min size???

    inliers->reserve(residuals.size());

    // Find median of square residuals
    std::vector<double> sq_residuals(residuals.size());
    std::transform(residuals.cbegin(), residuals.cend(), sq_residuals.begin(),
                   [](const auto& res) { return res * res; });

    std::nth_element(sq_residuals.begin(),
                     sq_residuals.begin() + sq_residuals.size() / 2,
                     sq_residuals.end());
    auto median = sq_residuals[sq_residuals.size() / 2];

    if ((sq_residuals.size() % 2) != 0) {
        std::nth_element(sq_residuals.begin(),
                         sq_residuals.begin() + (sq_residuals.size() / 2) - 1,
                         sq_residuals.end());
        median = 0.5 * (sq_residuals[(sq_residuals.size() / 2) - 1] + median);
    }

    // Find inliers
    // The threshold used here is a heuristic value learned from OpenCV.
    // See opencv/modules/calib3d/src/ptsetreg.cpp
    const auto threshold = 2.5 * 1.4826 *
                           (1 + 5. / (residuals.size() - _minSampleSize)) *
                           std::sqrt(median);
    const auto sq_threshold = threshold * threshold;

    for (size_t i{0}; i < residuals.size(); ++i) {
        if ((residuals[i] * residuals[i]) < sq_threshold) {
            inliers->emplace_back(i);
        }
    }

    return median;
}

} // namespace tl
