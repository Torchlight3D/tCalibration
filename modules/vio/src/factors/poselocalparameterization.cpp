#include "poselocalparameterization.h"

#include <Eigen/Geometry>

namespace tl {

using Eigen::Quaterniond;
using Eigen::Vector3d;

bool PoseLocalParameterization::Plus(const double *x, const double *delta,
                                     double *x_plus_delta) const
{
    Eigen::Map<const Vector3d> _p(x);
    Eigen::Map<const Quaterniond> _q(x + 3);
    Eigen::Map<const Vector3d> dp(delta);
    Quaterniond dq = Utility::deltaQ(Vector3d(delta + 3));

    // p' = p + delta_p
    Eigen::Map<Vector3d>{x_plus_delta} = _p + dp;
    // q' = q * delta_q
    Eigen::Map<Quaterniond>{x_plus_delta + 3} = (_q * dq).normalized();

    return true;
}

bool PoseLocalParameterization::PlusJacobian(const double *x,
                                             double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}

bool PoseLocalParameterization::Minus(const double *y, const double *x,
                                      double *y_minus_x) const
{
    return true;
}

bool PoseLocalParameterization::MinusJacobian(const double *x,
                                              double *jacobian) const
{
    return true;
}

} // namespace tl
