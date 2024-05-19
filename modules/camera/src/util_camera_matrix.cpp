#include "util_camera_matrix.h"

#include <glog/logging.h>

#include <tMath/Eigen/Utils>

namespace tl {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;

Eigen::Matrix3d intrinsicsToCalibrationMatrix(double fx, double skew,
                                              double y_x, double cx, double cy)
{
    Matrix3d K;
    // clang-format off
    K << fx,     skew,  cx,
         0., fx * y_x,  cy,
         0.,       0.,  1.;
    // clang-format on
    return K;
}

void calibrationMatrixToIntrinsics(const Eigen::Matrix3d& K, double* fx,
                                   double* skew, double* y_x, double* cx,
                                   double* cy)
{
    const auto& scale = K(2, 2);
    CHECK_NE(scale, 0.);

    *fx = K(0, 0) / scale;
    *skew = K(0, 1) / scale;
    *y_x = K(1, 1) / K(0, 0);
    *cx = K(0, 2) / scale;
    *cy = K(1, 2) / scale;
}

bool decomposeProjectionMatrix(const Matrix34d& P, Eigen::Matrix3d& K,
                               Eigen::Vector3d& rvec, Eigen::Vector3d& tvec)
{
    math::RQDecomposition<Matrix3d> rq{P.block<3, 3>(0, 0)};

    Matrix3d rmat = projectToRotationMatrix(rq.matrixQ());

    const double k_det = rq.matrixR().determinant();
    if (k_det == 0) {
        return false;
    }

    if (k_det > 0) {
        K = rq.matrixR();
    }
    else {
        K = -rq.matrixR();
    }

    // Fix the matrix such that all internal parameters are greater than 0.
    for (int i = 0; i < 3; ++i) {
        if (K(i, i) < 0) {
            K.col(i) *= -1.0;
            rmat.row(i) *= -1.0;
        }
    }

    // Solve for t.
    const Vector3d t = K.triangularView<Eigen::Upper>().solve(P.col(3));

    // c = - R' * t, and flip the sign according to k_det;
    if (k_det > 0) {
        tvec = -rmat.transpose() * t;
    }
    else {
        tvec = rmat.transpose() * t;
    }

    const AngleAxisd aa{rmat};
    rvec = aa.angle() * aa.axis();

    return true;
}

bool composeProjectionMatrix(const Eigen::Matrix3d& K,
                             const Eigen::Vector3d& rvec,
                             const Eigen::Vector3d& tvec, Matrix34d& P)
{
    const double angle = rvec.norm();
    // TODO: use isApprox0
    if (angle == 0) {
        P.block<3, 3>(0, 0) = Matrix3d::Identity();
    }
    else {
        P.block<3, 3>(0, 0) =
            AngleAxisd{angle, rvec / angle}.toRotationMatrix();
    }

    P.col(3) = -(P.block<3, 3>(0, 0) * tvec);
    P = K * P;

    return true;
}

Eigen::Matrix3d projectToRotationMatrix(const Eigen::Matrix3d& matrix)
{
    Eigen::JacobiSVD<Matrix3d> svd{matrix,
                                   Eigen::ComputeFullU | Eigen::ComputeFullV};

    Matrix3d rmat = svd.matrixU() * (svd.matrixV().transpose());
    // Valid rotation matrices should have a positive determinant.
    if (rmat.determinant() < 0) {
        rmat *= -1.0;
    }

    return rmat;
}

} // namespace tl
