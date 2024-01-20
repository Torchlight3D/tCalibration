#include "lmed_quality_measurement.h"

#include <tCore/ContainerUtils>

namespace tl {

namespace {

double calcMedianOfSquaredResiduals(const std::vector<double>& residuals)
{
    std::vector<double> squared_residuals;
    squared_residuals.reserve(residuals.size());
    std::transform(residuals.cbegin(), residuals.cend(),
                   squared_residuals.begin(),
                   [](const auto& residual) { return residual * residual; });

    return utils::FindMedian(squared_residuals);
}

void calcInliers(const std::vector<double>& residuals, double median,
                 int min_num_samples, std::vector<int>* inliers)
{
    // The median holds a squared residual. Thus, we take the squared root.
    // The threshold calculated here is computed based on a heuristic that
    // OpenCV uses. See modules/calib3d/src/ptsetreg.cpp
    const double inlier_threshold =
        2.5 * 1.4826 * (1 + 5.0 / (residuals.size() - min_num_samples)) *
        std::sqrt(median);
    const double squared_inlier_threshold = inlier_threshold * inlier_threshold;

    int i{0};
    for (const auto& residual : residuals) {
        if ((residual * residual) < squared_inlier_threshold) {
            inliers->emplace_back(i);
        }
        ++i;
    }
}

} // namespace

LmedQualityMeasurement::LmedQualityMeasurement(int minSampleSize)
    : QualityMeasurement(), min_sample_size_(minSampleSize)
{
}

double LmedQualityMeasurement::ComputeCost(const std::vector<double>& residuals,
                                           std::vector<int>* inliers)
{
    inliers->reserve(residuals.size());
    const double median = calcMedianOfSquaredResiduals(residuals);
    calcInliers(residuals, median, min_sample_size_, inliers);
    return median;
}

} // namespace tl
