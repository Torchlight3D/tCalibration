#pragma once

#include "quality_measurement.h"

namespace thoht {

// Assess quality of data by whether each error residual is less than an error
// threshold. If it is below the threshold, it is considered an inlier.
class InlierSupport : public QualityMeasurement
{
public:
    using QualityMeasurement::QualityMeasurement;

    double ComputeCost(const std::vector<double>& residuals,
                       std::vector<int>* inliers) override
    {
        inliers->reserve(residuals.size());
        for (size_t i{0}; i < residuals.size(); i++) {
            if (residuals[i] < error_thresh_) {
                inliers->emplace_back(static_cast<int>(i));
            }
        }

        return static_cast<double>(residuals.size() - inliers->size());
    }
};

} // namespace thoht
