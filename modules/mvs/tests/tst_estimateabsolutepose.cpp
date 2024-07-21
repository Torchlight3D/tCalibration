#include <ceres/rotation.h>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tMath/Eigen/Utils>
#include <tMath/Ransac/SampleConsensus>
#include <tMvs/Feature>
#include <tMvs/PnP/EstimateAbsolutePoseWithOrientation>

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
constexpr double kErrorThreshold =
    (kReprojectionError * kReprojectionError) / (kFocalLength * kFocalLength);

RandomNumberGenerator kRNG(66);
} // namespace

void ExecuteRandomTest1(const SacParameters& options,
                        const Eigen::Matrix3d& rotation,
                        const Eigen::Vector3d& position, double inlier_ratio,
                        double noise, double tolerance)
{
    // Create feature corrs (inliers and outliers) and add noise if
    // appropriate.
    std::vector<Feature2D3D> corrs;
    for (int i = 0; i < kNumPoints; i++) {
        Feature2D3D corr;
        corr.world_point =
            Vector3d{kRNG.randFloat(-2., 2.), kRNG.randFloat(-2., 2.),
                     kRNG.randFloat(6., 10.)};

        // Add an inlier or outlier.
        if (i < inlier_ratio * kNumPoints) {
            // Make sure the point is in front of the camera.
            corr.feature =
                (rotation * (corr.world_point - position)).hnormalized();
        }
        else {
            corr.feature = Vector2d::Random();
        }

        corrs.push_back(corr);
    }

    if (noise) {
        for (int i = 0; i < kNumPoints; i++) {
            AddNoiseToVector2(noise / kFocalLength, &corrs[i].feature);
        }
    }

    // Convert the camera rotation to angle axis.
    Vector3d R_c;
    ceres::RotationMatrixToAngleAxis(
        ceres::ColumnMajorAdapter3x3(rotation.data()), R_c.data());

    // Estimate the absolute pose.
    Vector3d position_est;
    SacSummary ransac_summary;
    EXPECT_TRUE(EstimateAbsolutePoseWithKnownOrientation(
        options, RansacType::RANSAC, R_c, corrs, &position_est,
        &ransac_summary));

    // Expect that the inlier ratio is close to the ground truth.
    EXPECT_GT(static_cast<double>(ransac_summary.inliers.size()), 3);

    // Expect poses are near.
    EXPECT_TRUE(math::ArraysEqualUpToScale(3, position.data(),
                                           position_est.data(), tolerance));
}

TEST(EstimateAbsolutePoseWithKnownOrientation, AllInliersNoNoise)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    const double kInlierRatio = 1.0;
    const double kNoise = 0.0;
    const double kPoseTolerance = 1e-4;

    const std::vector<Matrix3d> rotations{
        Matrix3d::Identity(),
        AngleAxisd{math::degToRad(12.), Vector3d::UnitY()}.toRotationMatrix(),
        AngleAxisd{math::degToRad(-9.), Vector3d{1., 0.2, -0.8}.normalized()}
            .toRotationMatrix()};
    const std::vector positions{Vector3d{-1.3, 0., 0.}, Vector3d{0., 0., 0.5}};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            ExecuteRandomTest1(options, rotation, position, kInlierRatio,
                               kNoise, kPoseTolerance);
        }
    }
}

TEST(EstimateAbsolutePoseWithKnownOrientation, AllInliersWithNoise)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    const double kInlierRatio = 1.0;
    const double kNoise = 1.0;
    const double kPoseTolerance = 1e-2;

    const std::vector<Matrix3d> rotations{
        Matrix3d::Identity(),
        AngleAxisd{math::degToRad(12.), Vector3d::UnitY()}.toRotationMatrix(),
        AngleAxisd{math::degToRad(-9.), Vector3d{1., 0.2, -0.8}.normalized()}
            .toRotationMatrix()};
    const std::vector positions{Vector3d{-1.3, 0., 0.}, Vector3d{0., 0., 0.5}};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            ExecuteRandomTest1(options, rotation, position, kInlierRatio,
                               kNoise, kPoseTolerance);
        }
    }
}

TEST(EstimateAbsolutePoseWithKnownOrientation, OutliersNoNoise)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    const double kInlierRatio = 0.7;
    const double kNoise = 0.0;
    const double kPoseTolerance = 1e-2;

    const std::vector<Matrix3d> rotations{Matrix3d::Identity(),
                                          RandomRotation(10.)};
    const std::vector positions{Vector3d::UnitX(), Vector3d::UnitY()};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            ExecuteRandomTest1(options, rotation, position, kInlierRatio,
                               kNoise, kPoseTolerance);
        }
    }
}

TEST(EstimateAbsolutePoseWithKnownOrientation, OutliersWithNoise)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    const double kInlierRatio = 0.7;
    const double kNoise = 1.0;
    const double kPoseTolerance = 1e-2;

    const std::vector<Matrix3d> rotations{Matrix3d::Identity(),
                                          RandomRotation(10.)};
    const std::vector positions{Vector3d::UnitX(), Vector3d::UnitY()};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            ExecuteRandomTest1(options, rotation, position, kInlierRatio,
                               kNoise, kPoseTolerance);
        }
    }
}
