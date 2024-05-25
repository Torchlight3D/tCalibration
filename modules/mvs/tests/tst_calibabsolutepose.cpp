#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tMath/Eigen/Utils>
#include <tMath/RANSAC/SampleConsensus>
#include <tMvs/PnP/EstimateCalibratedAbsolutePose>
#include <tMvs/Feature>
#include <tMvs/Types>

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
constexpr int kMinIterations = 50;
constexpr double kErrorThreshold =
    (kReprojectionError * kReprojectionError) / (kFocalLength * kFocalLength);

RandomNumberGenerator kRNG{66};
} // namespace

void ExecuteRandomTest(const SacParameters& options, const Matrix3d& rotation,
                       const Vector3d& position, double inlier_ratio,
                       double noise, double tolerance, PnPType pnp_type)
{
    // Create feature correspondences (inliers and outliers) and add noise if
    // appropriate.
    std::vector<Feature2D3D> correspondences;
    for (int i = 0; i < kNumPoints; i++) {
        Feature2D3D correspondence;
        correspondence.world_point =
            Vector3d(kRNG.RandDouble(-2.0, 2.0), kRNG.RandDouble(-2.0, 2.0),
                     kRNG.RandDouble(6.0, 10.0));

        // Add an inlier or outlier.
        if (i < inlier_ratio * kNumPoints) {
            // Make sure the point is in front of the camera.
            correspondence.feature =
                (rotation * (correspondence.world_point - position))
                    .hnormalized();
        }
        else {
            correspondence.feature = Vector2d::Random();
        }
        correspondences.emplace_back(correspondence);
    }

    if (noise) {
        for (int i = 0; i < kNumPoints; i++) {
            AddNoiseToProjection(noise / kFocalLength, &kRNG,
                                 &correspondences[i].feature);
        }
    }

    // Estimate the absolute pose.
    CalibratedAbsolutePose pose;
    SacSummary ransac_summary;
    EXPECT_TRUE(EstimateCalibratedAbsolutePose(options, RansacType::RANSAC,
                                               pnp_type, correspondences, &pose,
                                               &ransac_summary));

    // Expect that the inlier ratio is close to the ground truth.
    EXPECT_GT(static_cast<double>(ransac_summary.inliers.size()), 3);

    // Expect poses are near.
    EXPECT_TRUE(math::ArraysEqualUpToScale(9, rotation.data(),
                                           pose.rotation.data(), tolerance));
    EXPECT_TRUE(math::ArraysEqualUpToScale(3, position.data(),
                                           pose.position.data(), tolerance));
}

TEST(EstimateCalibratedAbsolutePose, AllInliersNoNoiseKNEIP)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    options.min_iterations = kMinIterations;
    const double kInlierRatio = 1.0;
    const double kNoise = 0.0;
    const double kPoseTolerance = 1e-4;
    PnPType type = PnPType::KNEIP;

    const std::vector<Matrix3d> rotations = {
        Matrix3d::Identity(),
        AngleAxisd(math::degToRad(12.0), Vector3d::UnitY()).toRotationMatrix(),
        AngleAxisd(math::degToRad(-9.0), Vector3d(1.0, 0.2, -0.8).normalized())
            .toRotationMatrix()};
    const std::vector<Vector3d> positions = {Vector3d(-1.3, 0, 0),
                                             Vector3d(0, 0, 0.5)};
    for (size_t i = 0; i < rotations.size(); i++) {
        for (size_t j = 0; j < positions.size(); j++) {
            ExecuteRandomTest(options, rotations[i], positions[j], kInlierRatio,
                              kNoise, kPoseTolerance, type);
        }
    }
}

TEST(EstimateCalibratedAbsolutePose, AllInliersWithNoiseKNEIP)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    options.min_iterations = kMinIterations;
    const double kInlierRatio = 1.0;
    const double kNoise = 1.0;
    const double kPoseTolerance = 1e-2;
    const PnPType type = PnPType::KNEIP;

    const std::vector<Matrix3d> rotations = {
        Matrix3d::Identity(),
        AngleAxisd(math::degToRad(12.0), Vector3d::UnitY()).toRotationMatrix(),
        AngleAxisd(math::degToRad(-9.0), Vector3d(1.0, 0.2, -0.8).normalized())
            .toRotationMatrix()};
    const std::vector<Vector3d> positions = {Vector3d(-1.3, 0, 0),
                                             Vector3d(0, 0, 0.5)};

    for (size_t i = 0; i < rotations.size(); i++) {
        for (size_t j = 0; j < positions.size(); j++) {
            ExecuteRandomTest(options, rotations[i], positions[j], kInlierRatio,
                              kNoise, kPoseTolerance, type);
        }
    }
}

TEST(EstimateCalibratedAbsolutePose, AllInliersWithNoiseDLS)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    options.min_iterations = kMinIterations;
    const double kInlierRatio = 1.0;
    const double kNoise = 1.0;
    const double kPoseTolerance = 1e-2;
    const PnPType type = PnPType::DLS;

    const std::vector<Matrix3d> rotations = {
        Matrix3d::Identity(),
        AngleAxisd(math::degToRad(12.0), Vector3d::UnitY()).toRotationMatrix(),
        AngleAxisd(math::degToRad(-9.0), Vector3d(1.0, 0.2, -0.8).normalized())
            .toRotationMatrix()};
    const std::vector<Vector3d> positions = {Vector3d(-1.3, 0, 0),
                                             Vector3d(0, 0, 0.5)};

    for (size_t i = 0; i < rotations.size(); i++) {
        for (size_t j = 0; j < positions.size(); j++) {
            ExecuteRandomTest(options, rotations[i], positions[j], kInlierRatio,
                              kNoise, kPoseTolerance, type);
        }
    }
}

TEST(EstimateCalibratedAbsolutePose, AllInliersWithNoiseSQPnP)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    options.min_iterations = kMinIterations;
    const double kInlierRatio = 1.0;
    const double kNoise = 1.0;
    const double kPoseTolerance = 1e-2;
    const PnPType type = PnPType::SQPnP;

    const std::vector<Matrix3d> rotations = {
        Matrix3d::Identity(),
        AngleAxisd(math::degToRad(12.0), Vector3d::UnitY()).toRotationMatrix(),
        AngleAxisd(math::degToRad(-9.0), Vector3d(1.0, 0.2, -0.8).normalized())
            .toRotationMatrix()};
    const std::vector<Vector3d> positions = {Vector3d(-1.3, 0, 0),
                                             Vector3d(0, 0, 0.5)};

    for (size_t i = 0; i < rotations.size(); i++) {
        for (size_t j = 0; j < positions.size(); j++) {
            ExecuteRandomTest(options, rotations[i], positions[j], kInlierRatio,
                              kNoise, kPoseTolerance, type);
        }
    }
}

TEST(EstimateCalibratedAbsolutePose, OutliersNoNoiseKNEIP)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    options.min_iterations = kMinIterations;
    const double kInlierRatio = 0.7;
    const double kNoise = 0.0;
    const double kPoseTolerance = 1e-2;
    const PnPType type = PnPType::KNEIP;

    const std::vector<Matrix3d> rotations = {Matrix3d::Identity(),
                                             RandomRotation(10.)};
    const std::vector<Vector3d> positions = {Vector3d(1, 0, 0),
                                             Vector3d(0, 1, 0)};

    for (size_t i = 0; i < rotations.size(); i++) {
        for (size_t j = 0; j < positions.size(); j++) {
            ExecuteRandomTest(options, rotations[i], positions[j], kInlierRatio,
                              kNoise, kPoseTolerance, type);
        }
    }
}

TEST(EstimateCalibratedAbsolutePose, OutliersWithNoiseKNEIP)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    options.min_iterations = kMinIterations;
    const double kInlierRatio = 0.7;
    const double kNoise = 1.0;
    const double kPoseTolerance = 1e-2;
    const PnPType type = PnPType::KNEIP;

    const std::vector<Matrix3d> rotations = {Matrix3d::Identity(),
                                             RandomRotation(10.)};
    const std::vector<Vector3d> positions = {Vector3d(1, 0, 0),
                                             Vector3d(0, 1, 0)};

    for (size_t i = 0; i < rotations.size(); i++) {
        for (size_t j = 0; j < positions.size(); j++) {
            ExecuteRandomTest(options, rotations[i], positions[j], kInlierRatio,
                              kNoise, kPoseTolerance, type);
        }
    }
}

TEST(EstimateCalibratedAbsolutePose, OutliersWithNoiseDLS)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    options.min_iterations = kMinIterations;
    const double kInlierRatio = 0.7;
    const double kNoise = 1.0;
    const double kPoseTolerance = 1e-2;
    const PnPType type = PnPType::DLS;

    const std::vector<Matrix3d> rotations = {Matrix3d::Identity(),
                                             RandomRotation(10.)};
    const std::vector<Vector3d> positions = {Vector3d(1, 0, 0),
                                             Vector3d(0, 1, 0)};

    for (size_t i = 0; i < rotations.size(); i++) {
        for (size_t j = 0; j < positions.size(); j++) {
            ExecuteRandomTest(options, rotations[i], positions[j], kInlierRatio,
                              kNoise, kPoseTolerance, type);
        }
    }
}

TEST(EstimateCalibratedAbsolutePose, OutliersWithNoiseSQPNP)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    options.min_iterations = kMinIterations;
    const double kInlierRatio = 0.7;
    const double kNoise = 1.0;
    const double kPoseTolerance = 1e-2;
    const PnPType type = PnPType::SQPnP;

    const std::vector<Matrix3d> rotations = {Matrix3d::Identity(),
                                             RandomRotation(10.0)};
    const std::vector<Vector3d> positions = {Vector3d(1, 0, 0),
                                             Vector3d(0, 1, 0)};

    for (size_t i = 0; i < rotations.size(); i++) {
        for (size_t j = 0; j < positions.size(); j++) {
            ExecuteRandomTest(options, rotations[i], positions[j], kInlierRatio,
                              kNoise, kPoseTolerance, type);
        }
    }
}

TEST(EstimateCalibratedAbsolutePose, OutliersWithNoiseKNEIP_LO)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    options.use_lo = true;
    options.lo_start_iterations = 5;
    options.min_iterations = kMinIterations;
    const double kInlierRatio = 0.7;
    const double kNoise = 1.0;
    const double kPoseTolerance = 1e-4;
    const PnPType type = PnPType::KNEIP;

    const std::vector<Matrix3d> rotations = {Matrix3d::Identity(),
                                             RandomRotation(10.0)};
    const std::vector<Vector3d> positions = {Vector3d(1, 0, 0),
                                             Vector3d(0, 1, 0)};

    for (size_t i = 0; i < rotations.size(); i++) {
        for (size_t j = 0; j < positions.size(); j++) {
            ExecuteRandomTest(options, rotations[i], positions[j], kInlierRatio,
                              kNoise, kPoseTolerance, type);
        }
    }
}

TEST(EstimateCalibratedAbsolutePose, OutliersWithNoiseDLS_LO)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    options.use_lo = true;
    options.lo_start_iterations = 5;
    options.min_iterations = kMinIterations;

    constexpr double kInlierRatio = 0.7;
    constexpr double kNoise = 1.0;
    constexpr double kPoseTolerance = 1e-4;
    constexpr PnPType type = PnPType::DLS;

    const std::vector<Matrix3d> rotations{Matrix3d::Identity(),
                                          RandomRotation(10.)};
    const std::vector<Vector3d> positions{Vector3d::UnitX(), Vector3d::UnitY()};

    for (size_t i = 0; i < rotations.size(); i++) {
        for (size_t j = 0; j < positions.size(); j++) {
            ExecuteRandomTest(options, rotations[i], positions[j], kInlierRatio,
                              kNoise, kPoseTolerance, type);
        }
    }
}

TEST(EstimateCalibratedAbsolutePose, OutliersWithNoiseSQPNP_LO)
{
    SacParameters options;
    options.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    options.use_mle = true;
    options.error_thresh = kErrorThreshold;
    options.failure_probability = 0.001;
    options.use_lo = true;
    options.lo_start_iterations = 5;
    options.min_iterations = kMinIterations;
    constexpr double kInlierRatio = 0.7;
    constexpr double kNoise = 1.0;
    constexpr double kPoseTolerance = 1e-4;
    constexpr PnPType type = PnPType::SQPnP;

    const std::vector<Matrix3d> rotations{Matrix3d::Identity(),
                                          RandomRotation(10.0)};
    const std::vector<Vector3d> positions{Vector3d::UnitX(), Vector3d::UnitY()};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            ExecuteRandomTest(options, rotation, position, kInlierRatio, kNoise,
                              kPoseTolerance, type);
        }
    }
}
