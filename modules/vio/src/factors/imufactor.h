#pragma once

#include <ceres/sized_cost_function.h>

namespace tl {

class IntegrationBase;

class IMUFactor final : public ceres::SizedCostFunction<15, 7, 9, 7, 9>
{
public:
    explicit IMUFactor(IntegrationBase *_pre_integration);

    bool Evaluate(double const *const *parameters, double *residuals,
                  double **jacobians) const override;

public:
    IntegrationBase *pre_integration;
};

} // namespace tl
