#include <ceres/rotation.h>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <tCore/Math>
#include <tMvs/Poses/EstimateRotationNonlinear>

using namespace tl;

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace {

// TODO: Move to global utils
Eigen::Matrix3d EulerAnglesToMatrix(double x, double y, double z)
{
    return (AngleAxisd{math::degToRad(x), Vector3d::UnitX()} *
            AngleAxisd{math::degToRad(y), Vector3d::UnitY()} *
            AngleAxisd{math::degToRad(z), Vector3d::UnitZ()})
        .toRotationMatrix();
}

} // namespace

void PairwiseRotationErrorTest(const Eigen::Matrix3d& R,
                               const Eigen::Matrix3d& g_R1,
                               const Eigen::Matrix3d& g_R2, double weight = 1.)
{
    Vector3d rvec, g_rvec1, rotation2;
    ceres::RotationMatrixToAngleAxis(R.data(), rvec.data());
    ceres::RotationMatrixToAngleAxis(g_R1.data(), g_rvec1.data());
    ceres::RotationMatrixToAngleAxis(g_R2.data(), rotation2.data());

    // Compute ground truth angular error.
    const AngleAxisd loop_rotation{g_R2 * g_R1.transpose()};
    const Matrix3d R_err = loop_rotation * R.transpose();

    Vector3d rvec_err;
    ceres::RotationMatrixToAngleAxis(R_err.data(), rvec_err.data());
    rvec_err *= weight;

    // Initialize error function and compute rotation error.
    const PairwiseRotationError pairWiseError{rvec, weight};
    Vector3d rvec_err_est;
    pairWiseError(g_rvec1.data(), rotation2.data(), rvec_err_est.data());

    constexpr double kTolerance = 1e-12;
    EXPECT_NEAR(rvec_err_est(0), rvec_err(0), kTolerance);
    EXPECT_NEAR(rvec_err_est(1), rvec_err(1), kTolerance);
    EXPECT_NEAR(rvec_err_est(2), rvec_err(2), kTolerance);
}

TEST(EstimateRotationNonlinear, PairwiseRotationError_SmallRotation)
{
    const Matrix3d g_R1 = Matrix3d::Identity();
    const Matrix3d g_R2 =
        AngleAxisd{math::degToRad(2.), Vector3d::UnitZ()}.toRotationMatrix();
    const Matrix3d R =
        AngleAxisd{math::degToRad(1.), Vector3d::UnitZ()}.toRotationMatrix();

    PairwiseRotationErrorTest(R, g_R1, g_R2);
}

TEST(EstimateRotationNonlinear, PairwiseRotationError_NontrivialRotation)
{
    const Matrix3d g_R1 = Matrix3d::Identity();
    const Matrix3d g_R2 = EulerAnglesToMatrix(5.3, 1.2, 8.1);
    const Matrix3d R = EulerAnglesToMatrix(5.9, 1.8, 7.6);

    PairwiseRotationErrorTest(R, g_R1, g_R2);
}

TEST(EstimateRotationNonlinear, PairwiseRotationError_Rotation180)
{
    const Matrix3d g_R1 = Matrix3d::Identity();
    const Matrix3d g_R2 =
        AngleAxisd{math::degToRad(179.), Vector3d::UnitZ()}.toRotationMatrix();
    const Matrix3d R =
        AngleAxisd{math::degToRad(-179.), Vector3d::UnitZ()}.toRotationMatrix();

    PairwiseRotationErrorTest(R, g_R1, g_R2);
    PairwiseRotationErrorTest(R, g_R2, g_R1);
}

TEST(EstimateRotationNonlinear, PairwiseRotationError_Weight)
{
    const Matrix3d g_R1 = Matrix3d::Identity();
    const Matrix3d g_R2 = EulerAnglesToMatrix(5.3, 1.2, 8.1);
    const Matrix3d R = EulerAnglesToMatrix(5.9, 1.8, 7.6);

    PairwiseRotationErrorTest(R, g_R1, g_R2, 2.);
}
