#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tMath/EigenUtils>
#include <tMath/RansacCreator>
#include <tMvs/EstimateUncalibratedAbsolutePose>
#include <tMvs/FeatureCorrespondence>

#include "test_utils.h"

using namespace tl;

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {
constexpr int kNumPoints = 100;
constexpr double kFocalLength = 1000.0;
constexpr double kReprojectionError = 4.0;
constexpr double kErrorThreshold = kReprojectionError * kReprojectionError;

RandomNumberGenerator rng(64);
} // namespace

inline void ExecuteRandomTest(const SacParameters& options,
                              const Matrix3d& rotation,
                              const Vector3d& position, double inlier_ratio,
                              double noise, double tolerance)
{
    // Create feature correspondences (inliers and outliers) and add noise if
    // appropriate.
    std::vector<FeatureCorrespondence2D3D> correspondences;
    for (int i = 0; i < kNumPoints; i++) {
        FeatureCorrespondence2D3D correspondence;
        correspondence.world_point =
            Vector3d(rng.RandDouble(-2.0, 2.0), rng.RandDouble(-2.0, 2.0),
                     rng.RandDouble(6.0, 10.0));

        // Add an inlier or outlier.
        if (i < inlier_ratio * kNumPoints) {
            // Make sure the point is in front of the camera.
            correspondence.feature =
                kFocalLength *
                ((rotation * (correspondence.world_point - position))
                     .hnormalized());
        }
        else {
            correspondence.feature =
                kFocalLength *
                Vector2d(rng.RandDouble(-1.0, 1.0), rng.RandDouble(-1.0, 1.0));
        }
        correspondences.emplace_back(correspondence);
    }

    if (noise) {
        for (int i = 0; i < kNumPoints; i++) {
            AddNoiseToProjection(noise, &rng, &correspondences[i].feature);
        }
    }

    // Estimate the absolute pose.
    UncalibratedAbsolutePose pose;
    SacSummary ransac_summary;
    EXPECT_TRUE(EstimateUncalibratedAbsolutePose(
        options, RansacType::RANSAC, correspondences, &pose, &ransac_summary));

    // Expect that the inlier ratio is close to the ground truth.
    EXPECT_GT(static_cast<double>(ransac_summary.inliers.size()), 3);

    // Expect poses are near.
    EXPECT_TRUE(math::ArraysEqualUpToScale(9, rotation.data(),
                                           pose.rotation.data(), tolerance));
    // The position is more noisy than the rotation usually.
    EXPECT_TRUE(math::ArraysEqualUpToScale(
        3, position.data(), pose.position.data(), 2.0 * tolerance));

    // Expect focal length is near.
    constexpr double kFocalLengthTolerance = 0.05;
    EXPECT_NEAR(pose.focal_length, kFocalLength,
                kFocalLengthTolerance * kFocalLength);
}

TEST(EstimateUncalibratedAbsolutePose, AllInliersNoNoise)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(rng);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    constexpr double kInlierRatio = 1.0;
    constexpr double kNoise = 0.0;
    constexpr double kPoseTolerance = 1e-4;

    const std::vector<Matrix3d> rotations = {
        Matrix3d::Identity(),
        AngleAxisd(math::degToRad(12.0), Vector3d::UnitY()).toRotationMatrix(),
        AngleAxisd(math::degToRad(-9.0), Vector3d(1.0, 0.2, -0.8).normalized())
            .toRotationMatrix()};
    const std::vector positions = {Vector3d(-1.3, 0, 0), Vector3d(0, 0, 0.5)};
    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            ExecuteRandomTest(options, rotation, position, kInlierRatio, kNoise,
                              kPoseTolerance);
        }
    }
}

TEST(EstimateUncalibratedAbsolutePose, AllInliersWithNoise)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(rng);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    options.max_iterations = 1000;
    constexpr double kInlierRatio = 1.0;
    constexpr double kNoise = 1.0;
    constexpr double kPoseTolerance = 1e-2;

    const std::vector<Matrix3d> rotations = {
        Matrix3d::Identity(),
        AngleAxisd(math::degToRad(12.0), Vector3d::UnitY()).toRotationMatrix(),
        AngleAxisd(math::degToRad(-9.0), Vector3d(1.0, 0.2, -0.8).normalized())
            .toRotationMatrix()};
    const std::vector positions = {Vector3d(-1.3, 0, 0), Vector3d(0, 0, 0.5)};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            ExecuteRandomTest(options, rotation, position, kInlierRatio, kNoise,
                              kPoseTolerance);
        }
    }
}

TEST(EstimateUncalibratedAbsolutePose, OutliersNoNoise)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(rng);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    options.max_iterations = 1000;
    constexpr double kInlierRatio = 0.7;
    constexpr double kNoise = 0.0;
    constexpr double kPoseTolerance = 1e-2;

    const std::vector<Matrix3d> rotations = {Matrix3d::Identity(),
                                             RandomRotation(15.)};
    const std::vector positions = {Vector3d(1, 0, 0), Vector3d(0, 1, 0)};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            ExecuteRandomTest(options, rotation, position, kInlierRatio, kNoise,
                              kPoseTolerance);
        }
    }
}

TEST(EstimateUncalibratedAbsolutePose, OutliersWithNoise)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(rng);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    options.max_iterations = 1000;
    constexpr double kInlierRatio = 0.7;
    constexpr double kNoise = 1.0;
    constexpr double kPoseTolerance = 0.1;

    const std::vector<Matrix3d> rotations = {Matrix3d::Identity(),
                                             RandomRotation(15.)};
    const std::vector positions = {Vector3d(1, 0, 0), Vector3d(0, 1, 0)};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            ExecuteRandomTest(options, rotation, position, kInlierRatio, kNoise,
                              kPoseTolerance);
        }
    }
}
