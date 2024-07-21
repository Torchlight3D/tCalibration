#pragma once

#include <memory>

#include <ceres/loss_function.h>

#include <tMath/Types>

namespace tl {

class TruncatedLoss final : public ceres::LossFunction
{
public:
    explicit TruncatedLoss(double a);

    void Evaluate(double sq_norm, double out[3]) const override;

private:
    const double squared_error_;
};

std::unique_ptr<ceres::LossFunction> createLossFunction(LossFunctionType type,
                                                        double scale);

} // namespace tl
