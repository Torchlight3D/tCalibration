#include "initialposefactor.h"

#include "../utils/utility.h"

using Eigen::Quaterniond;
using Eigen::Vector3d;

using Vector6d = Eigen::Vector<double, 6>;

InitialPoseFactor::InitialPoseFactor(const Eigen::Vector3d &_P,
                                     const Eigen::Quaterniond &_Q)
    : init_P(_P), init_Q(_Q), sqrt_info(1000. * Matrix6d::Identity())
{
}

bool InitialPoseFactor::Evaluate(const double *const *parameters,
                                 double *residuals, double **jacobians) const
{
    Eigen::Map<const Vector3d> P{parameters[0]};
    Eigen::Map<const Quaterniond> Q{parameters[0] + 3};
    const auto Qinv_Q = init_Q.inverse() * Q;

    Eigen::Map<Vector6d> res{residuals};
    res.block<3, 1>(0, 0) = P - init_P;
    res.block<3, 1>(3, 0) = 2 * Qinv_Q.vec();
    res = sqrt_info * res;

    if (jacobians) {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>>
                jacobian_pose(jacobians[0]);
            jacobian_pose.setZero();
            jacobian_pose.block<3, 3>(0, 0).setIdentity();
            jacobian_pose.block<3, 3>(3, 3) =
                Utility::Qleft(Qinv_Q).bottomRightCorner<3, 3>();
            jacobian_pose = sqrt_info * jacobian_pose;
        }
    }

    return true;
}
