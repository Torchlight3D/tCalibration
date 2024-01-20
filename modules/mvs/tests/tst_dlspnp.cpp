#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <tCore/tGlobal>
#include <tCore/ContainerUtils>
#include <tMath/MathBase>
#include <tMath/RandomGenerator>
#include <tMvs/DlsPnP>
#include "test_utils.h"

using namespace tl;
using namespace tl::math;

using Eigen::AngleAxisd;
using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;

RandomNumberGenerator rng(59);

inline void TestDlsPnpWithNoise(const Vector3dList& world_points,
                                double projection_noise_std_dev,
                                const Quaterniond& expected_rotation,
                                const Vector3d& expected_translation,
                                double max_reprojection_error,
                                double max_rotation_difference,
                                double max_translation_difference)
{
    const int num_points = world_points.size();

    Matrix34d expected_transform;
    expected_transform << expected_rotation.toRotationMatrix(),
        expected_translation;

    Vector2dList feature_points;
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
            AddNoiseToProjection(projection_noise_std_dev, &rng,
                                 &feature_points[i]);
        }
    }

    // Run DLS PnP.
    QuaterniondList soln_rotation;
    Vector3dList soln_translation;
    DlsPnp(feature_points, world_points, soln_rotation, soln_translation);

    // Check solutions and verify at least one is close to the actual solution.
    const int num_solutions = soln_rotation.size();
    EXPECT_GT(num_solutions, 0);
    bool matched_transform = false;
    for (int i = 0; i < num_solutions; i++) {
        // Check that reprojection errors are small.
        Matrix34d soln_transform;
        soln_transform << soln_rotation[i].toRotationMatrix(),
            soln_translation[i];

        for (int j = 0; j < num_points; j++) {
            const Vector2d reprojected_point =
                (soln_transform * world_points[j].homogeneous())
                    .eval()
                    .hnormalized();
            const double reprojection_error =
                (feature_points[j] - reprojected_point).squaredNorm();
            ASSERT_LE(reprojection_error, max_reprojection_error);
        }

        // Check that the solution is accurate.
        const double rotation_difference =
            expected_rotation.angularDistance(soln_rotation[i]);
        const bool matched_rotation =
            (rotation_difference < max_rotation_difference);
        const double translation_difference =
            (expected_translation - soln_translation[i]).squaredNorm();
        const bool matched_translation =
            (translation_difference < max_translation_difference);

        if (matched_translation && matched_rotation) {
            matched_transform = true;
        }
    }
    EXPECT_TRUE(matched_transform);
}

inline void BasicTest()
{
    const Vector3dList points_3d{
        Vector3d(-1.0, 3.0, 3.0), Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0), Vector3d(2.0, 1.0, 3.0)};
    const Quaterniond soln_rotation{
        AngleAxisd{degToRad(13.), Vector3d::UnitZ()}};
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    constexpr double kNoise = 0.0;
    constexpr double kMaxReprojectionError = 1e-4;
    constexpr double kMaxAllowedRotationDifference = 1e-5;
    constexpr double kMaxAllowedTranslationDifference = 1e-8;

    TestDlsPnpWithNoise(points_3d, kNoise, soln_rotation, soln_translation,
                        kMaxReprojectionError, kMaxAllowedRotationDifference,
                        kMaxAllowedTranslationDifference);
}

TEST(DlsPnp, Basic) { BasicTest(); }

TEST(DlsPnp, NoiseTest)
{
    const Vector3dList points_3d = {
        Vector3d(-1.0, 3.0, 3.0),  Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0),  Vector3d(2.0, 1.0, 3.0),
        Vector3d(-1.0, -3.0, 2.0), Vector3d(1.0, -2.0, 1.0),
        Vector3d(-1.0, 4.0, 2.0),  Vector3d(-2.0, 2.0, 3.0)};
    const Quaterniond soln_rotation{
        AngleAxisd{degToRad(13.), Vector3d::UnitZ()}};
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    constexpr double kNoise = 1.0 / 512.0;
    constexpr double kMaxReprojectionError = 5e-3;
    constexpr double kMaxAllowedRotationDifference = degToRad(0.25);
    constexpr double kMaxAllowedTranslationDifference = 1e-2;

    TestDlsPnpWithNoise(points_3d, kNoise, soln_rotation, soln_translation,
                        kMaxReprojectionError, kMaxAllowedRotationDifference,
                        kMaxAllowedTranslationDifference);
}

TEST(DlsPnp, ManyPoints)
{
    // Sets some test rotations and translations.
    static const Vector3d kAxes[]{Vector3d::UnitZ(),
                                  Vector3d::UnitY(),
                                  Vector3d::UnitX(),
                                  Vector3d(1.0, 0.0, 1.0).normalized(),
                                  Vector3d(0.0, 1.0, 1.0).normalized(),
                                  Vector3d(1.0, 1.0, 1.0).normalized(),
                                  Vector3d(0.0, 1.0, 1.0).normalized(),
                                  Vector3d(1.0, 1.0, 1.0).normalized()};

    static const double kRotationAngles[utils::ArraySize(kAxes)] = {
        degToRad(7.0),  degToRad(12.0), degToRad(15.0),
        degToRad(20.0), degToRad(11.0),
        degToRad(0.0), // Tests no rotation.
        degToRad(5.0),
        degToRad(0.0) // Tests no rotation and no translation.
    };

    static const Vector3d kTranslations[utils::ArraySize(kAxes)] = {
        Vector3d(1.0, 1.0, 1.0),  Vector3d(3.0, 2.0, 13.0),
        Vector3d(4.0, 5.0, 11.0), Vector3d(1.0, 2.0, 15.0),
        Vector3d(3.0, 1.5, 18.0), Vector3d(1.0, 7.0, 11.0),
        Vector3d(0.0, 0.0, 0.0), // Tests no translation.
        Vector3d(0.0, 0.0, 0.0)  // Tests no translation and no rotation.
    };

    constexpr int num_points[3]{100, 500, 1000};
    constexpr double kNoise{1. / 512.};
    constexpr double kMaxReprojectionError{1e-2};
    constexpr double kMaxAllowedRotationDifference = degToRad(0.3);
    constexpr double kMaxAllowedTranslationDifference{5e-3};

    for (size_t i{0}; i < utils::ArraySize(kAxes); i++) {
        const Quaterniond soln_rotation(
            AngleAxisd(kRotationAngles[i], kAxes[i]));
        for (size_t j{0}; j < utils::ArraySize(num_points); j++) {
            Vector3dList points_3d;
            points_3d.reserve(num_points[j]);
            for (int k = 0; k < num_points[j]; k++) {
                points_3d.push_back(Vector3d(rng.RandDouble(-5., 5.),
                                             rng.RandDouble(-5., 5.),
                                             rng.RandDouble(2., 10.)));
            }

            TestDlsPnpWithNoise(points_3d, kNoise, soln_rotation,
                                kTranslations[i], kMaxReprojectionError,
                                kMaxAllowedRotationDifference,
                                kMaxAllowedTranslationDifference);
        }
    }
}

TEST(DlsPnp, NoRotation)
{
    const Vector3dList points_3d{Vector3d(-1., 3., 3.),  Vector3d(1., -1., 2.),
                                 Vector3d(-1., 1., 2.),  Vector3d(2., 1.0, 3.),
                                 Vector3d(-1., -3., 2.), Vector3d(1., -2., 1.),
                                 Vector3d(-1., 4., 2.),  Vector3d(-2., 2., 3.)};
    const Quaterniond soln_rotation{
        AngleAxisd{degToRad(0.), Vector3d::UnitZ()}};
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    constexpr double kNoise = 1.0 / 512.0;
    constexpr double kMaxReprojectionError = 5e-3;
    constexpr double kMaxAllowedRotationDifference = degToRad(0.25);
    constexpr double kMaxAllowedTranslationDifference = 5e-4;

    TestDlsPnpWithNoise(points_3d, kNoise, soln_rotation, soln_translation,
                        kMaxReprojectionError, kMaxAllowedRotationDifference,
                        kMaxAllowedTranslationDifference);
}

TEST(DlsPnp, NoTranslation)
{
    const Vector3dList points_3d{Vector3d(-1., 3., 3.),  Vector3d(1., -1., 2.),
                                 Vector3d(-1., 1., 2.),  Vector3d(2., 1., 3.),
                                 Vector3d(-1., -3., 2.), Vector3d(1., -2., 1.),
                                 Vector3d(-1., 4., 2.),  Vector3d(-2., 2., 3.)};
    const Quaterniond soln_rotation{
        AngleAxisd{degToRad(13.), Vector3d::UnitZ()}};
    const Vector3d soln_translation(0.0, 0.0, 0.0);
    constexpr double kNoise{1.0 / 512.0};
    constexpr double kMaxReprojectionError{1e-2};
    constexpr double kMaxAllowedRotationDifference = degToRad(0.2);
    constexpr double kMaxAllowedTranslationDifference{5e-3};

    TestDlsPnpWithNoise(points_3d, kNoise, soln_rotation, soln_translation,
                        kMaxReprojectionError, kMaxAllowedRotationDifference,
                        kMaxAllowedTranslationDifference);
}

TEST(DlsPnp, OrthogonalRotation)
{
    const Vector3dList points_3d{Vector3d(-1., 3., 3.),  Vector3d(1., -1., 2.),
                                 Vector3d(-1., 1., 2.),  Vector3d(2., 1., 3.),
                                 Vector3d(-1., -3., 2.), Vector3d(1., -2., 1.),
                                 Vector3d(-1., 4., 2.),  Vector3d(-2., 2., 3.)};
    const Quaterniond soln_rotation{
        AngleAxisd{degToRad(90.), Vector3d::UnitZ()}};
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    constexpr double kNoise{1.0 / 512.0};
    constexpr double kMaxReprojectionError{5e-3};
    constexpr double kMaxAllowedRotationDifference = degToRad(0.25);
    constexpr double kMaxAllowedTranslationDifference{5e-3};

    TestDlsPnpWithNoise(points_3d, kNoise, soln_rotation, soln_translation,
                        kMaxReprojectionError, kMaxAllowedRotationDifference,
                        kMaxAllowedTranslationDifference);
}
