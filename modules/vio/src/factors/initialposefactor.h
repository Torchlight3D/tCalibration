#pragma once

#include <ceres/sized_cost_function.h>
#include <Eigen/Geometry>

class InitialPoseFactor final : public ceres::SizedCostFunction<6, 7>
{
    using Matrix6d = Eigen::Matrix<double, 6, 6>;

public:
    InitialPoseFactor(const Eigen::Vector3d &_P, const Eigen::Quaterniond &_Q);

    bool Evaluate(double const *const *parameters, double *residuals,
                  double **jacobians) const override;

public:
    const Eigen::Vector3d init_P;
    const Eigen::Quaterniond init_Q;
    const Matrix6d sqrt_info;
};
