#include <ceres/rotation.h>
#include <Eigen/LU>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <tCamera/Camera>
#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tMvs/Feature>
#include <tMvs/Poses/OptimizeRelativePosition>

#include "test_utils.h"

using namespace tl;

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace {
RandomNumberGenerator kRNG(52);

Camera RandomCamera()
{
    Camera camera;
    camera.setPosition(Vector3d::Random());
    camera.setOrientationFromAngleAxis(0.2 * Vector3d::Random());
    camera.setImageSize(1000, 1000);
    camera.setFocalLength(800);
    camera.setPrincipalPoint(500.0, 500.0);
    return camera;
}

void GetRelativeTranslationFromCameras(const Camera& camera1,
                                       const Camera& camera2,
                                       Eigen::Vector3d* relative_position)
{
    const Vector3d rotated_relative_position =
        camera2.position() - camera1.position();
    ceres::AngleAxisRotatePoint(camera1.orientationAsAngleAxis().data(),
                                rotated_relative_position.data(),
                                relative_position->data());
    relative_position->normalize();
}

void TestOptimization(const Camera& camera1, const Camera& camera2,
                      const std::vector<Eigen::Vector3d>& world_points,
                      double kPixelNoise, double kTranslationNoise,
                      double kTolerance)
{
    // Project points and create feature correspondences.
    std::vector<Feature2D2D> matches;
    for (int i = 0; i < world_points.size(); i++) {
        const Vector4d point = world_points[i].homogeneous();
        Feature2D2D match;
        camera1.projectPoint(point, match.feature1.pos);
        camera2.projectPoint(point, match.feature2.pos);
        AddNoiseToVector2(kPixelNoise, &match.feature1.pos);
        AddNoiseToVector2(kPixelNoise, &match.feature2.pos);

        // Undo the calibration.
        match.feature1.pos =
            camera1.pixelToNormalizedCoordinates(match.feature1.pos)
                .hnormalized();
        match.feature2.pos =
            camera2.pixelToNormalizedCoordinates(match.feature2.pos)
                .hnormalized();
        matches.emplace_back(match);
    }

    Vector3d relative_position;
    GetRelativeTranslationFromCameras(camera1, camera2, &relative_position);

    const Vector3d gt_relative_position = relative_position;

    // Add noise to relative translation.
    const AngleAxisd translation_noise(
        math::degToRad(kRNG.randNorm(0.0, kTranslationNoise)),
        Vector3d::Random().normalized());
    relative_position = translation_noise * relative_position;

    CHECK(OptimizeRelativePositionWithKnownRotation(
        matches, camera1.orientationAsAngleAxis(),
        camera2.orientationAsAngleAxis(), &relative_position));

    const double translation_error = math::radToDeg(std::acos(
        std::clamp(gt_relative_position.dot(relative_position), -1.0, 1.0)));
    EXPECT_LT(translation_error, kTolerance)
        << "GT Position = " << gt_relative_position.transpose()
        << "\nEstimated position = " << relative_position.transpose();
}

} // namespace

TEST(OptimizeRelativePositionWithKnownRotationTest, NoNoise)
{
    constexpr double kTolerance = 1e-6;
    constexpr double kPixelNoise = 0.0;
    constexpr double kTranslationNoise = 0.0;
    constexpr int kNumPoints = 100;
    std::vector<Vector3d> points(kNumPoints);

    // Set up random points.
    for (int i = 0; i < kNumPoints; i++) {
        Vector3d point(kRNG.randFloat(-2.0, 2.0), kRNG.randFloat(-2.0, -2.0),
                       kRNG.randFloat(8.0, 10.0));
        points[i] = point;
    }

    // Set up random cameras.
    Camera camera1 = RandomCamera();
    Camera camera2 = RandomCamera();
    camera2.setPosition(camera2.position().normalized());
    TestOptimization(camera1, camera2, points, kPixelNoise, kTranslationNoise,
                     kTolerance);
}

TEST(OptimizeRelativePositionWithKnownRotationTest, PixelNoise)
{
    constexpr double kTolerance = 2.0;
    constexpr double kPixelNoise = 1.0;
    constexpr double kTranslationNoise = 0.0;
    constexpr int kNumPoints = 100;
    std::vector<Vector3d> points(kNumPoints);

    // Set up random points.
    for (int i = 0; i < kNumPoints; i++) {
        Vector3d point(kRNG.randFloat(-2.0, 2.0), kRNG.randFloat(-2.0, -2.0),
                       kRNG.randFloat(8.0, 10.0));
        points[i] = point;
    }

    // Set up random cameras.
    Camera camera1 = RandomCamera();
    Camera camera2 = RandomCamera();
    camera2.setPosition(camera2.position().normalized());
    TestOptimization(camera1, camera2, points, kPixelNoise, kTranslationNoise,
                     kTolerance);
}

TEST(OptimizeRelativePositionWithKnownRotationTest, TranslationNoise)
{
    constexpr double kTolerance = 2.0;
    constexpr double kPixelNoise = 0.0;
    constexpr double kTranslationNoise = 5.0;
    constexpr int kNumPoints = 100;
    std::vector<Vector3d> points(kNumPoints);

    // Set up random points.
    for (int i = 0; i < kNumPoints; i++) {
        const Vector3d point(kRNG.randFloat(-2.0, 2.0),
                             kRNG.randFloat(-2.0, -2.0),
                             kRNG.randFloat(8.0, 10.0));
        points[i] = point;
    }

    // Set up random cameras.
    Camera camera1 = RandomCamera();
    Camera camera2 = RandomCamera();
    camera2.setPosition(camera2.position().normalized());
    TestOptimization(camera1, camera2, points, kPixelNoise, kTranslationNoise,
                     kTolerance);
}

TEST(OptimizeRelativePositionWithKnownRotationTest, PixelAndTranslationNoise)
{
    constexpr double kTolerance = 5.0;
    constexpr double kPixelNoise = 1.0;
    constexpr double kTranslationNoise = 5.0;
    constexpr int kNumPoints = 100;
    std::vector<Vector3d> points(kNumPoints);

    // Set up random points.
    for (int i = 0; i < kNumPoints; i++) {
        Vector3d point(kRNG.randFloat(-2.0, 2.0), kRNG.randFloat(-2.0, -2.0),
                       kRNG.randFloat(8.0, 10.0));
        points[i] = point;
    }

    // Set up random cameras.
    Camera camera1 = RandomCamera();
    Camera camera2 = RandomCamera();
    camera2.setPosition(camera2.position().normalized());
    TestOptimization(camera1, camera2, points, kPixelNoise, kTranslationNoise,
                     kTolerance);
}
