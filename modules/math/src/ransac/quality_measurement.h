#pragma once

#include <vector>

namespace thoht {

class QualityMeasurement
{
public:
    QualityMeasurement() = default;
    explicit QualityMeasurement(double error_thres) : error_thresh_(error_thres)
    {
    }
    virtual ~QualityMeasurement() = default;

    //  Must be called before Compare is called.
    virtual bool Initialize() { return true; }

    // Given the residuals, assess a quality metric for the data.
    // The return is a cost, the lower the better.
    // The indicies of the inliers are additionally returned.
    virtual double ComputeCost(const std::vector<double>& residuals,
                               std::vector<int>* inliers) = 0;

protected:
    double error_thresh_;
};

} // namespace thoht
