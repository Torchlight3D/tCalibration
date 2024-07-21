#include "rotation.h"

#include <ceres/rotation.h>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <tCore/Math>
#include <tCore/RandomGenerator>

namespace tl {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;

using Matrix43d = Eigen::Matrix<double, 4, 3>;

Eigen::MatrixXd ProjectToSOd(const Eigen::MatrixXd& M)
{
    Eigen::JacobiSVD<MatrixXd> svd{M,
                                   Eigen::ComputeFullU | Eigen::ComputeFullV};

    const auto detU = svd.matrixU().determinant();
    const auto detV = svd.matrixV().determinant();

    if (detU * detV > 0) {
        return svd.matrixU() * svd.matrixV().transpose();
    }

    MatrixXd U_prime = svd.matrixU();
    U_prime.col(U_prime.cols() - 1) *= -1;
    return U_prime * svd.matrixV().transpose();
}

Eigen::Vector3d RelativeRotationFromTwoRotations(
    const Eigen::Vector3d& rvec1, const Eigen::Vector3d& rvec2,
    const std::optional<double>& noise)
{
    const Matrix3d extra = noise.has_value()
                               ? AngleAxisd{math::degToRad(noise.value()),
                                            Vector3d::Random().normalized()}
                                     .toRotationMatrix()
                               : Matrix3d::Identity();

    Matrix3d R1, R2;
    ceres::AngleAxisToRotationMatrix(rvec1.data(), R1.data());
    ceres::AngleAxisToRotationMatrix(rvec2.data(), R2.data());

    const AngleAxisd aa{extra * R2 * R1.transpose()};
    return aa.angle() * aa.axis();
}

Eigen::Vector3d RelativeTranslationFromTwoPositions(
    const Eigen::Vector3d& tvec1, const Eigen::Vector3d& tvec2,
    const Eigen::Vector3d& rvec1, const std::optional<double>& noise)
{
    const auto extra = noise.has_value()
                           ? AngleAxisd{math::degToRad(noise.value()),
                                        Vector3d::Random().normalized()}
                           : AngleAxisd::Identity();

    Matrix3d R1;
    ceres::AngleAxisToRotationMatrix(rvec1.data(), R1.data());
    const Vector3d t = R1 * (tvec2 - tvec1).normalized();
    return extra * t;
}

Eigen::Vector3d MultiplyRotations(const Eigen::Vector3d& rvec1,
                                  const Eigen::Vector3d& rvec2)
{
    Matrix3d R1, R2;
    ceres::AngleAxisToRotationMatrix(rvec1.data(), R1.data());
    ceres::AngleAxisToRotationMatrix(rvec2.data(), R2.data());

    const Matrix3d R = R1 * R2;

    Vector3d rvec;
    ceres::RotationMatrixToAngleAxis(R.data(), rvec.data());

    return rvec;
}

Eigen::Vector3d ApplyRelativeRotation(const Eigen::Vector3d& rvec1,
                                      const Eigen::Vector3d& rvec_21)
{
    Vector3d rvec2;
    Matrix3d R1, R_21;
    ceres::AngleAxisToRotationMatrix(rvec1.data(),
                                     ceres::ColumnMajorAdapter3x3(R1.data()));
    ceres::AngleAxisToRotationMatrix(rvec_21.data(),
                                     ceres::ColumnMajorAdapter3x3(R_21.data()));

    const Matrix3d R2 = R_21 * R1;
    ceres::RotationMatrixToAngleAxis(ceres::ColumnMajorAdapter3x3(R2.data()),
                                     rvec2.data());
    return rvec2;
}

// We use the reprojection error constraint:
//
//   rotated_feature = [x - cx; y - cy] / (z - cz)
//
// where the 3D point is [x y z]^t and the unknown position is [cx cy cz].
// This constraint can be rearranged into a linear system:
//
//   [1  0  -u] * c = [x - u * z]
//   [0  1  -v]     = [y - v * z]
//
// where rotated_feature = [u v]. By stacking this constraint for both features
// we obtain a 4x3 linear system whose solution is the camera position.
bool PositionFromTwoRays(const Eigen::Vector2d& rotated_feature1,
                         const Eigen::Vector3d& point1,
                         const Eigen::Vector2d& rotated_feature2,
                         const Eigen::Vector3d& point2,
                         Eigen::Vector3d* position)
{
    CHECK_NOTNULL(position);

    // Create the left hand side of the linear system above.
    Matrix43d lhs;
    lhs.block<2, 2>(0, 0).setIdentity();
    lhs.block<2, 2>(2, 0).setIdentity();
    lhs.block<2, 1>(0, 2) = -rotated_feature1;
    lhs.block<2, 1>(2, 2) = -rotated_feature2;

    // Create the right hand side of the linear system above.
    Vector4d rhs;
    rhs.head<2>() = point1.head<2>() - rotated_feature1 * point1.z();
    rhs.tail<2>() = point2.head<2>() - rotated_feature2 * point2.z();

    Eigen::ColPivHouseholderQR<Matrix43d> solver(lhs);
    // If the linear solver is not well-conditioned then the position cannot be
    // reliably computed.
    if (solver.rank() != 3) {
        return false;
    }

    *position = solver.solve(rhs);
    return true;
}

} // namespace tl
