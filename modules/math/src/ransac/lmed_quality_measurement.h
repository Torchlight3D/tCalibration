#pragma once

#include <algorithm>
#include <cmath>

#include "quality_measurement.h"

namespace thoht {

// The idea of Least Median of Squares Regression (LMed) is to find a hypothesis
// that minimizes the median of the squared residuals.

// Reference:
// P.Rousseeuw, "Least Median of Squares Regression," Journal of the American
// statistical association, 1984.
class LmedQualityMeasurement : public QualityMeasurement
{
public:
    explicit LmedQualityMeasurement(int minSampleSize);

    // The cost is the squared residual. LMed minimizes the median of the
    // squared residuals over the hypotheses.
    double ComputeCost(const std::vector<double>& residuals,
                       std::vector<int>* inliers) override;

private:
    const int min_sample_size_;
};

} // namespace thoht
