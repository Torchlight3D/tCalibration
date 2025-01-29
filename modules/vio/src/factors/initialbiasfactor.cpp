#include "initialbiasfactor.h"

using Eigen::Vector3d;

using Vector6d = Eigen::Vector<double, 6>;

InitialBiasFactor::InitialBiasFactor(const Eigen::Vector3d &_Ba,
                                     const Eigen::Vector3d &_Bg)
    : init_Ba(_Ba),
      init_Bg(_Bg),
      sqrt_info(1.0 / (0.001) * Matrix6d::Identity())
{
}

bool InitialBiasFactor::Evaluate(const double *const *parameters,
                                 double *residuals, double **jacobians) const
{
    Eigen::Map<Vector6d> res{residuals};
    res.block<3, 1>(0, 0) =
        Eigen::Map<const Vector3d>{parameters[0] + 3} - init_Ba;
    res.block<3, 1>(3, 0) =
        Eigen::Map<const Vector3d>{parameters[0] + 6} - init_Bg;
    res = sqrt_info * res;

    if (jacobians) {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 6, 9, Eigen::RowMajor>>
                jacobian_bias{jacobians[0]};
            jacobian_bias.setZero();
            jacobian_bias.block<3, 3>(0, 3).setIdentity();
            jacobian_bias.block<3, 3>(3, 6).setIdentity();
            jacobian_bias = sqrt_info * jacobian_bias;
        }
    }

    return true;
}
