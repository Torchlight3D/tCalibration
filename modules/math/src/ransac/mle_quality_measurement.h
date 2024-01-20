#pragma once

#include "quality_measurement.h"

namespace tl {

class MLEQualityMeasurement : public QualityMeasurement
{
public:
    using QualityMeasurement::QualityMeasurement;

    double ComputeCost(const std::vector<double>& residuals,
                       std::vector<int>* inliers) override
    {
        inliers->reserve(residuals.size());
        double score{0.};
        for (size_t i{0}; i < residuals.size(); i++) {
            if (residuals[i] < error_thresh_) {
                score += residuals[i];
                inliers->emplace_back(static_cast<int>(i));
            }
            else {
                score += error_thresh_;
            }
        }
        return score;
    }
};

} // namespace tl
