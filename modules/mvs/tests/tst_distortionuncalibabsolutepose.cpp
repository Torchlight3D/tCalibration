#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <tCamera/DivisionUndistortionCameraModel>
#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tMath/Eigen/Utils>
#include <tMath/RANSAC/SampleConsensus>
#include <tMvs/PnP/EstimateRadialDistortionUncalibratedAbsolutePose>
#include <tMvs/Feature>

#include "test_utils.h"

using namespace tl;

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace {
constexpr int kNumPoints = 20;
constexpr double kFocalLength = 1000.;
constexpr double kRadialDistortion = -1e-7;
constexpr double kReprojectionError = 1.;
constexpr double kErrorThreshold = kReprojectionError * kReprojectionError;

RandomNumberGenerator kRNG{64};

// TODO: Duplicated code in DivisionUndistortionCameraModel and
// EstimateRadialDistortionUncalibratedAbsolutePose, but whatever
inline void DistortPoint(const Eigen::Vector2d& point2d, double distortion,
                         Eigen::Vector2d* distorted_point)
{
    const double r_u_sq = point2d[0] * point2d[0] + point2d[1] * point2d[1];

    const double denom = 2.0 * distortion * r_u_sq;
    const double inner_sqrt = 1.0 - 4.0 * distortion * r_u_sq;

    // If the denominator is nearly zero then we can evaluate the distorted
    // coordinates as k or r_u^2 goes to zero. Both evaluate to the identity.
    if (std::abs(denom) < 1e-15 || inner_sqrt < 0.0) {
        (*distorted_point)[0] = point2d[0];
        (*distorted_point)[1] = point2d[1];
    }
    else {
        const double scale = (1.0 - std::sqrt(inner_sqrt)) / denom;
        (*distorted_point)[0] = point2d[0] * scale;
        (*distorted_point)[1] = point2d[1] * scale;
    }
}

} // namespace

inline void ExecuteRandomTest(const SacParameters& options,
                              const Matrix3d& gt_rotation,
                              const Vector3d& gt_translation,
                              double inlier_ratio, double noise,
                              double tolerance)
{
    // Create feature correspondences (inliers and outliers) and add noise if
    // appropriate.
    std::vector<Feature2D3D> corres;
    for (int i = 0; i < kNumPoints; i++) {
        Feature2D3D corr;
        corr.world_point =
            Vector3d(kRNG.RandDouble(-5., 5.), kRNG.RandDouble(-5., 5.),
                     kRNG.RandDouble(3., 10.));

        const Matrix3d K =
            Vector3d(kFocalLength, kFocalLength, 1.0).asDiagonal();
        Matrix34d gt_transformation;
        gt_transformation << gt_rotation, gt_translation;
        const Matrix34d gt_projection = K * gt_transformation;

        // Add an inlier or outlier.
        if (i < inlier_ratio * kNumPoints) {
            // Make sure the point is in front of the camera.
            corr.feature =
                (gt_projection * corr.world_point.homogeneous()).hnormalized();
            DistortPoint(corr.feature, kRadialDistortion, &corr.feature);
        }
        else {
            corr.feature =
                Vector2d(kRNG.RandDouble(-1., 1.), kRNG.RandDouble(-1., 1.));
        }

        corres.emplace_back(corr);
    }

    if (noise > 0.) {
        for (int i = 0; i < kNumPoints; i++) {
            AddNoiseToProjection(noise, &kRNG, &corres[i].feature);
        }
    }

    // Estimate the absolute pose.
    RadialDistUncalibratedAbsolutePose pose;
    RadialDistUncalibratedAbsolutePoseMetaData meta_data;
    meta_data.max_focal_length = 2000;
    meta_data.min_focal_length = 100;
    meta_data.min_radial_distortion = -1e-9;
    meta_data.max_radial_distortion = -1e-5;
    SacSummary ransac_summary;
    EXPECT_TRUE(EstimateRadialDistUncalibratedAbsolutePose(
        options, RansacType::RANSAC, corres, meta_data, &pose,
        &ransac_summary));

    // Expect poses are near.
    EXPECT_TRUE(math::ArraysEqualUpToScale(9, gt_rotation.data(),
                                           pose.rotation.data(), tolerance));
    // The position is more noisy than the rotation usually.
    EXPECT_TRUE(math::ArraysEqualUpToScale(
        3, gt_translation.data(), pose.translation.data(), 2.0 * tolerance));

    // Expect focal length is near.
    constexpr double kFocalLengthTolerance = 0.05;
    EXPECT_NEAR(pose.focal_length, kFocalLength,
                kFocalLengthTolerance * kFocalLength);
    // Expect radial distortion is near.
    EXPECT_NEAR(pose.radial_distortion, kRadialDistortion,
                std::abs(2.0 * kFocalLengthTolerance * kRadialDistortion));
}

TEST(EstimateRadialDistUncalibratedAbsolutePose, AllInliersNoNoise)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    options.min_iterations = 1;

    constexpr double kInlierRatio = 1.0;
    constexpr double kNoise = 0.0;
    constexpr double kPoseTolerance = 1e-4;

    const std::vector<Matrix3d> rotations{
        Matrix3d::Identity(),
        AngleAxisd(math::degToRad(12.), Vector3d::UnitY()).toRotationMatrix(),
        AngleAxisd(math::degToRad(-9.), Vector3d(1., 0.2, -0.8).normalized())
            .toRotationMatrix()};
    const std::vector<Vector3d> positions{Vector3d(1.3, 0., 0.),
                                          Vector3d(1., 1., 0.1)};
    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            ExecuteRandomTest(options, rotation, position, kInlierRatio, kNoise,
                              kPoseTolerance);
        }
    }
}

TEST(EstimateRadialDistUncalibratedAbsolutePose, AllInliersWithNoise)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    options.max_iterations = 1000;
    options.min_iterations = 1;

    constexpr double kInlierRatio = 1.0;
    constexpr double kNoise = 1.0;
    constexpr double kPoseTolerance = 1e-2;

    const Matrix3dList rotations{
        Matrix3d::Identity(),
        AngleAxisd{math::degToRad(12.), Vector3d::UnitY()}.toRotationMatrix(),
        AngleAxisd{math::degToRad(-9.), Vector3d{1.0, 0.2, -0.8}.normalized()}
            .toRotationMatrix()};
    const Vector3dList positions{Vector3d(1.3, 0., 0.),
                                 Vector3d(1.0, 1.0, 0.1)};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            ExecuteRandomTest(options, rotation, position, kInlierRatio, kNoise,
                              kPoseTolerance);
        }
    }
}

TEST(EstimateRadialDistUncalibratedAbsolutePose, OutliersNoNoise)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    options.max_iterations = 1000;
    options.min_iterations = 10;

    constexpr double kInlierRatio = 0.7;
    constexpr double kNoise = 0.0;
    constexpr double kPoseTolerance = 1e-2;

    const Matrix3dList rotations{Matrix3d::Identity(), RandomRotation(15.)};
    const Vector3dList positions{Vector3d(1.3, 0, 0.0),
                                 Vector3d(1.0, 1.0, 0.1)};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            ExecuteRandomTest(options, rotation, position, kInlierRatio, kNoise,
                              kPoseTolerance);
        }
    }
}

// FIXME: Failed this case
TEST(EstimateRadialDistUncalibratedAbsolutePose, OutliersWithNoise)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    options.max_iterations = 1000;
    options.min_iterations = 10;

    constexpr double kInlierRatio = 0.7;
    constexpr double kNoise = 1.0;
    constexpr double kPoseTolerance = 0.1;

    const Matrix3dList rotations{Matrix3d::Identity(), RandomRotation(15.)};
    const Vector3dList positions{Vector3d(1.3, 0, 0.0),
                                 Vector3d(1.0, 1.0, 0.1)};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            ExecuteRandomTest(options, rotation, position, kInlierRatio, kNoise,
                              kPoseTolerance);
        }
    }
}
