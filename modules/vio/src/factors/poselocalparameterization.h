#pragma once

#include <ceres/manifold.h>

namespace tl {

class PoseLocalParameterization final : public ceres::Manifold
{
public:
    bool Plus(const double *x, const double *delta,
              double *x_plus_delta) const override;
    bool PlusJacobian(const double *x, double *jacobian) const override;
    bool Minus(const double *y, const double *x,
               double *y_minus_x) const override;
    bool MinusJacobian(const double *x, double *jacobian) const override;

    int AmbientSize() const override { return 7; };
    int TangentSize() const override { return 6; };
};

} // namespace tl
