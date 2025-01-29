#pragma once

#include <ceres/sized_cost_function.h>
#include <Eigen/Core>

class InitialBiasFactor final : public ceres::SizedCostFunction<6, 9>
{
    using Matrix6d = Eigen::Matrix<double, 6, 6>;

public:
    InitialBiasFactor(const Eigen::Vector3d &_Ba, const Eigen::Vector3d &_Bg);

    bool Evaluate(const double *const *parameters, double *residuals,
                  double **jacobians) const override;

public:
    const Eigen::Vector3d init_Ba, init_Bg;
    const Matrix6d sqrt_info;
};
