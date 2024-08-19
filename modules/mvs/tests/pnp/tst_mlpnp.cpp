#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tMath/Eigen/Types>
#include <tMvs/PnP/MLPnP>

#include "../test_utils.h"

using namespace tl;

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {
RandomNumberGenerator rng(55);
}

void TestMLPnPWithNoise(const std::vector<Eigen::Vector3d>& world_points,
                        double projection_noise_std_dev,
                        const Eigen::Matrix3d& expected_rotation,
                        const Eigen::Vector3d& expected_translation,
                        double max_reprojection_error,
                        double max_rotation_difference,
                        double max_translation_difference)
{
    const int num_points = world_points.size();

    Matrix34d expected_transform;
    expected_transform << expected_rotation, expected_translation;

    std::vector<Vector2d> feature_points;
    feature_points.reserve(num_points);
    for (int i = 0; i < num_points; i++) {
        // Reproject 3D points into camera frame.
        feature_points.push_back(
            (expected_transform * world_points[i].homogeneous())
                .eval()
                .hnormalized());
    }

    if (projection_noise_std_dev) {
        // Adds noise to both of the rays.
        for (int i = 0; i < num_points; i++) {
            AddNoiseToVector2(projection_noise_std_dev, &feature_points[i]);
        }
    }

    // Run DLS PnP.
    Matrix3d soln_rotation;
    Vector3d soln_translation;
    MLPnP(feature_points, {}, world_points, &soln_rotation, &soln_translation);

    // Check solutions and verify at least one is close to the actual solution.
    bool matched_transform = false;

    Matrix34d soln_transform;
    soln_transform << soln_rotation, soln_translation;
    std::cout << expected_transform << std::endl;
    std::cout << soln_transform << std::endl;
    for (int j = 0; j < num_points; j++) {
        const Vector2d reprojected_point =
            (soln_transform * world_points[j].homogeneous())
                .eval()
                .hnormalized();
        const double reprojection_error =
            (feature_points[j] - reprojected_point).squaredNorm();
        ASSERT_LE(reprojection_error, max_reprojection_error);
    }
}

TEST(MLPnP, NoNoiseTest)
{
    const std::vector<Vector3d> points_3d = {
        Vector3d(-1.0, 3.0, 3.0),  Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0),  Vector3d(2.0, 1.0, 3.0),
        Vector3d(-1.0, -3.0, 2.0), Vector3d(1.0, -2.0, 1.0),
        Vector3d(-1.0, 4.0, 2.0),  Vector3d(-2.0, 2.0, 3.0)};
    const Matrix3d soln_rotation =
        AngleAxisd(math::degToRad(13.0), Vector3d(0.0, 0.0, 1.0))
            .toRotationMatrix();
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    const double kNoise = 0.0 / 512.0;
    const double kMaxReprojectionError = 5e-3;
    const double kMaxAllowedRotationDifference = math::degToRad(0.25);
    const double kMaxAllowedTranslationDifference = 1e-2;

    TestMLPnPWithNoise(points_3d, kNoise, soln_rotation, soln_translation,
                       kMaxReprojectionError, kMaxAllowedRotationDifference,
                       kMaxAllowedTranslationDifference);
}

// TEST(MLPnP, NoNoisePlanarTest)
// {
//     const std::vector<Vector3d> points_3d = {
//         Vector3d(-1.0, 3.0, 5.0),  Vector3d(1.0, -1.0, 5.0),
//         Vector3d(-1.0, 1.0, 5.0),  Vector3d(2.0, 1.0, 5.0),
//         Vector3d(-1.0, -3.0, 5.0), Vector3d(1.0, -2.0, 5.0),
//         Vector3d(-1.0, 4.0, 5.0),  Vector3d(-2.0, 2.0, 5.0)};
//     const Matrix3d soln_rotation =
//         AngleAxisd(math::degToRad(13.0), Vector3d(0.0, 0.0, 1.0))
//             .toRotationMatrix();
//     const Vector3d soln_translation(1.0, 1.0, 1.0);
//     const double kNoise = 0.0 / 512.0;
//     const double kMaxReprojectionError = 5e-3;
//     const double kMaxAllowedRotationDifference = math::degToRad(0.25);
//     const double kMaxAllowedTranslationDifference = 1e-2;

//     TestMLPnPWithNoise(points_3d, kNoise, soln_rotation, soln_translation,
//                        kMaxReprojectionError, kMaxAllowedRotationDifference,
//                        kMaxAllowedTranslationDifference);
// }

TEST(MLPnP, NoiseTest)
{
    const std::vector<Vector3d> points_3d = {
        Vector3d(-1.0, 3.0, 3.0),  Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0),  Vector3d(2.0, 1.0, 3.0),
        Vector3d(-1.0, -3.0, 2.0), Vector3d(1.0, -2.0, 1.0),
        Vector3d(-1.0, 4.0, 2.0),  Vector3d(-2.0, 2.0, 3.0)};
    const Matrix3d soln_rotation =
        AngleAxisd(math::degToRad(13.0), Vector3d(0.0, 0.0, 1.0))
            .toRotationMatrix();
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    const double kNoise = 0.5 / 512.0;
    const double kMaxReprojectionError = 5e-3;
    const double kMaxAllowedRotationDifference = math::degToRad(0.25);
    const double kMaxAllowedTranslationDifference = 1e-2;

    TestMLPnPWithNoise(points_3d, kNoise, soln_rotation, soln_translation,
                       kMaxReprojectionError, kMaxAllowedRotationDifference,
                       kMaxAllowedTranslationDifference);
}
