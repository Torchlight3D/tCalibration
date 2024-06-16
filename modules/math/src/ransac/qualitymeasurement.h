#pragma once

#include <vector>

namespace tl {

class QualityMeasurement
{
public:
    QualityMeasurement() = default;
    explicit QualityMeasurement(double threshold) : _threshold(threshold) {}
    virtual ~QualityMeasurement() = default;

    //  Must be called before Compare is called.
    virtual bool Initialize() { return true; }

    // Given the residuals, assess a quality metric for the data.
    // The return is a cost, the lower the better.
    // The indicies of the inliers are additionally returned.
    virtual double ComputeCost(const std::vector<double>& residuals,
                               std::vector<size_t>* inliers) = 0;

protected:
    double _threshold{std::numeric_limits<double>::max()};
};

// Brief:
// Default QualityMeasurement
// Assess quality of data by whether each error residual is less than an error
// threshold. If it is below the threshold, it is considered an inlier.
class InlierSupport final : public QualityMeasurement
{
public:
    using QualityMeasurement::QualityMeasurement;

    double ComputeCost(const std::vector<double>& residuals,
                       std::vector<size_t>* inliers) override;
};

class MLEQualityMeasurement final : public QualityMeasurement
{
public:
    using QualityMeasurement::QualityMeasurement;

    double ComputeCost(const std::vector<double>& residuals,
                       std::vector<size_t>* inliers) override;
};

// Brief:
// The idea of Least Median of Squares Regression (LMed) is to find a hypothesis
// that minimizes the median of the squared residuals.

// Reference:
// P.Rousseeuw, "Least Median of Squares Regression," Journal of the American
// statistical association, 1984.
class LmedQualityMeasurement final : public QualityMeasurement
{
public:
    explicit LmedQualityMeasurement(size_t minSampleSize);

    double ComputeCost(const std::vector<double>& residuals,
                       std::vector<size_t>* inliers) override;

private:
    const size_t _minSampleSize;
};

} // namespace tl
