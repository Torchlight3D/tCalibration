#include <glog/logging.h>
#include <gtest/gtest.h>

#include <tCore/ContainerUtils>
#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tMvs/Epipolar/Triangulation>
#include <tMvs/Feature>

#include "../test_utils.h"

using namespace tl;

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace {
RandomNumberGenerator kRNG{59};
}

class Triangulation : public ::testing::Test
{
protected:
    void SetUp() override
    {
        //
        _opts.noise.reset();
    }

    void TearDown() override {}

    void TestTwoViews(const Eigen::Vector3d& objPoint,
                      const Eigen::Quaterniond& rotation,
                      const Eigen::Vector3d& position)
    {
        Matrix34d pose1;
        pose1 << rotation.toRotationMatrix(), position.normalized();
        const Matrix34d pose2 = Matrix34d::Identity();

        // Reproject point into both image 2, assume image 1 is identity
        // rotation at the origin.
        Vector2d imgPoint1 =
            (pose1 * objPoint.homogeneous()).eval().hnormalized();
        Vector2d imgPoint2 =
            (pose2 * objPoint.homogeneous()).eval().hnormalized();

        // Add projection noise if required.
        if (_opts.noise.has_value()) {
            AddNoiseToVector2(_opts.noise.value(), &imgPoint1);
            AddNoiseToVector2(_opts.noise.value(), &imgPoint2);
        }

        // Triangulate with Optimal.
        Vector4d triangulated_point;
        EXPECT_TRUE(triangulateTwoViews(pose1, pose2, imgPoint1, imgPoint2,
                                        _opts.type, &triangulated_point));

        // Check the reprojection error.
        EXPECT_LE(ReprojectionError(pose1, triangulated_point, imgPoint1),
                  _ref.maxRpe);
        EXPECT_LE(ReprojectionError(pose2, triangulated_point, imgPoint2),
                  _ref.maxRpe);
    }

    void TestNViews()
    {
        constexpr int kNumViews = 8;

        // Sets some test rotations and translations.
        const std::vector kRotations{
            Quaterniond(AngleAxisd(math::degToRad(7.), Vector3d::UnitZ())),
            Quaterniond(AngleAxisd(math::degToRad(12.), Vector3d::UnitY())),
            Quaterniond(AngleAxisd(math::degToRad(15.), Vector3d::UnitX())),
            Quaterniond(AngleAxisd(math::degToRad(20.),
                                   Vector3d{1.0, 0.0, 1.0}.normalized())),
            Quaterniond(AngleAxisd(math::degToRad(11.),
                                   Vector3d(0.0, 1.0, 1.0).normalized())),
            Quaterniond(AngleAxisd(math::degToRad(0.),
                                   Vector3d(1.0, 1.0, 1.0).normalized())),
            Quaterniond(AngleAxisd(math::degToRad(5.),
                                   Vector3d(0.0, 1.0, 1.0).normalized())),
            Quaterniond(AngleAxisd(math::degToRad(0.),
                                   Vector3d(1.0, 1.0, 1.0).normalized()))};

        const std::vector
            kTranslations{
                Vector3d{1., 1., 1.}, {3., 2., 13.},  {4., 5., 11.},
                {1., 2., 15.},        {3., 1.5, 91.}, {1., 7., 11.},
                {0., 0., 0.}, // Tests no translation.
                {0., 0., 0.}  // Tests no translation and no rotation.
            };

        // Set up model points.
        const std::vector kObjectPoints{
            Vector3d{-1.62, -2.99, 6.12}, {4.42, -1.53, 9.83},
            {1.45, -0.59, 5.29},          {1.89, -1.10, 8.22},
            {-0.21, 2.38, 5.63},          {0.61, -0.97, 7.49},
            {0.48, 0.70, 8.94},           {1.65, -2.56, 8.63},
            {2.44, -0.20, 7.78},          {2.84, -2.58, 7.35},
            {-1.35, -2.84, 7.33},         {-0.42, 1.54, 8.86},
            {2.56, 1.72, 7.86},           {1.75, -1.39, 5.73},
            {2.08, -3.91, 8.37},          {-0.91, 1.36, 9.16},
            {2.84, 1.54, 8.74},           {-1.01, 3.02, 8.18},
            {-3.73, -0.62, 7.81},         {-2.98, -1.88, 6.23},
            {2.39, -0.19, 6.47},          {-0.63, -1.05, 7.11},
            {-1.76, -0.55, 5.18},         {-3.19, 3.27, 8.18},
            {0.31, -2.77, 7.54},          {0.54, -3.77, 9.77},
        };

        Matrix3d calibration;
        // clang-format off
        calibration << 800.,   0., 600.,
            0., 800., 400.,
            0.,   0.,   1.;
        // clang-format on

        // Set up pose matrices.
        std::vector<Matrix34d> poses(kNumViews);
        for (int i = 0; i < kNumViews; i++) {
            poses[i] << kRotations[i].toRotationMatrix(), kTranslations[i];
        }

        for (const auto& objPoint : kObjectPoints) {
            // Reproject model point into the images.
            std::vector<Vector2d> image_points(kNumViews);
            for (int i = 0; i < kNumViews; i++) {
                image_points[i] =
                    (poses[i] * objPoint.homogeneous()).eval().hnormalized();
            }

            // Add projection noise if required.
            if (_opts.noise.has_value()) {
                for (int i = 0; i < kNumViews; i++) {
                    AddNoiseToVector2(_opts.noise.value(), &image_points[i]);
                }
            }

            Vector4d triangulated_point;
            ASSERT_TRUE(
                TriangulateNView(poses, image_points, &triangulated_point));

            // Check the reprojection error.
            for (int i = 0; i < kNumViews; i++) {
                EXPECT_LE(ReprojectionError(poses[i], triangulated_point,
                                            image_points[i]),
                          _ref.maxRpe);
            }
        }
    }

protected:
    struct
    {
        TriangulationMethodType type;
        std::optional<double> noise;
    } _opts;

    struct
    {
        double maxRpe;
    } _ref;

private:
    static double ReprojectionError(const Matrix34d& pose,
                                    const Eigen::Vector4d& objPoint,
                                    const Eigen::Vector2d& imgPoint)
    {
        const Vector3d reproj = pose * objPoint;
        return (reproj.hnormalized() - imgPoint).squaredNorm();
    }
};

TEST_F(Triangulation, StandardNoNoise)
{
    _opts.type = TriangulationMethodType::STANDARD;
    _ref.maxRpe = 1e-12;

    const Quaterniond kRotation{AngleAxisd{0.15, Vector3d::UnitY()}};
    const Vector3d kTranslation{-3., 1.5, 11.};

    const std::vector objPoints{Vector3d{5., 20., 23.},
                                Vector3d{-6., 16., 33.}};
    for (const auto& objPoint : objPoints) {
        TestTwoViews(objPoint, kRotation, kTranslation);
    }
}

TEST_F(Triangulation, StandardWithNoise)
{
    _opts.type = TriangulationMethodType::STANDARD;
    _opts.noise = 1. / 512.;
    _ref.maxRpe = 1e-5;

    const Quaterniond kRotation{AngleAxisd{0.15, Vector3d::UnitY()}};
    const Vector3d kTranslation{-3., 1.5, 11.};

    const std::vector objPoints{Vector3d{5., 20., 23.},
                                Vector3d{-6., 16., 33.}};
    for (const auto& objPoint : objPoints) {
        TestTwoViews(objPoint, kRotation, kTranslation);
    }
}

TEST_F(Triangulation, DLTNoNoise)
{
    _opts.type = TriangulationMethodType::DLT;
    _ref.maxRpe = 1e-12;

    const Quaterniond kRotation{AngleAxisd{0.15, Vector3d::UnitY()}};
    const Vector3d kTranslation{-3., 1.5, 11.};

    const std::vector objPoints{Vector3d(5.0, 20.0, 23.0),
                                Vector3d(-6.0, 16.0, 33.0)};
    for (const auto& objPoint : objPoints) {
        TestTwoViews(objPoint, kRotation, kTranslation);
    }
}

TEST_F(Triangulation, DLTWithNoise)
{
    _opts.type = TriangulationMethodType::DLT;
    _opts.noise = 1. / 512.;
    _ref.maxRpe = 1e-5;

    const Quaterniond kRotation{AngleAxisd{0.15, Vector3d::UnitY()}};
    const Vector3d kTranslation{-3., 1.5, 11.};

    const Vector3d objPoints[2]{Vector3d{5., 20., 23.},
                                Vector3d{-6., 16., 33.}};
    for (const auto& objPoint : objPoints) {
        TestTwoViews(objPoint, kRotation, kTranslation);
    }
}

TEST_F(Triangulation, MidpointNoNoise)
{
    _opts.type = TriangulationMethodType::MIDPOINT;
    _ref.maxRpe = 1e-12;

    const Quaterniond kRotation{AngleAxisd{0.15, Vector3d::UnitY()}};
    const Vector3d kTranslation{-3., 1.5, 11.};

    const std::vector objPoints{Vector3d{5., 20., 23.},
                                Vector3d{-6., 16., 33.}};
    for (const auto& objPoint : objPoints) {
        TestTwoViews(objPoint, kRotation, kTranslation);
    }
}

TEST_F(Triangulation, MidpointWithNoise)
{
    _opts.type = TriangulationMethodType::MIDPOINT;
    _opts.noise = 1. / 512.;
    _ref.maxRpe = 1e-5;

    const Quaterniond kRotation{AngleAxisd{0.15, Vector3d::UnitY()}};
    const Vector3d kTranslation{-3., 1.5, 11.};

    const std::vector objPoints{Vector3d{5., 20., 23.},
                                Vector3d{-6., 16., 33.}};
    for (const auto& objPoint : objPoints) {
        TestTwoViews(objPoint, kRotation, kTranslation);
    }
}

TEST_F(Triangulation, NViewSVDNoNoise)
{
    _opts.type = TriangulationMethodType::SVD;
    _ref.maxRpe = 1e-2;

    TestNViews();
}

TEST_F(Triangulation, NViewSVDWithNoise)
{
    _opts.type = TriangulationMethodType::SVD;
    _opts.noise = 1. / 512.;
    _ref.maxRpe = 5e-4;

    TestNViews();
}

TEST_F(Triangulation, NViewL2MinNoNoise)
{
    GTEST_SKIP() << "Not implement...";

    _opts.type = TriangulationMethodType::L2_MINIMIZATION;
    _ref.maxRpe = 1e-2;

    TestNViews();
}

TEST_F(Triangulation, NViewL2MinWithNoise)
{
    GTEST_SKIP() << "Not implement...";

    _opts.type = TriangulationMethodType::L2_MINIMIZATION;
    _opts.noise = 1. / 512.;
    _ref.maxRpe = 5e-4;

    TestNViews();
}

bool TestIsTriangulatedPointInFrontOfCameras(const Eigen::Vector3d& point3d,
                                             const Eigen::Matrix3d& rotation,
                                             const Eigen::Vector3d& translation)
{
    Feature2D2D corr;
    corr.feature1.pos = point3d.hnormalized();
    corr.feature2.pos = (rotation * point3d + translation).hnormalized();
    const Vector3d position = -rotation.transpose() * translation;
    return IsTriangulatedPointInFrontOfCameras(corr, rotation, position);
}

TEST(TriangulationHelper, CheckPointInFrontOfCamera_AllFront)
{
    const Matrix3d rotation = Matrix3d::Identity();
    const Vector3d position{-1., 0., 0.};
    const Vector3d point{0., 0., 5.};
    EXPECT_TRUE(
        TestIsTriangulatedPointInFrontOfCameras(point, rotation, position));
}

TEST(TriangulationHelper, CheckPointInFrontOfCamera_AllBehind)
{
    const Matrix3d rotation = Matrix3d::Identity();
    const Vector3d position{-1., 0., 0.};
    const Vector3d point{0., 0., -5.};
    EXPECT_FALSE(
        TestIsTriangulatedPointInFrontOfCameras(point, rotation, position));
}

TEST(TriangulationHelper, CheckPointInFrontOfCamera_OneFrontOneBehind)
{
    const Matrix3d rotation = Matrix3d::Identity();
    const Vector3d position{0., 0., -2.};
    const Vector3d point{0., 0., 1.};
    EXPECT_FALSE(
        TestIsTriangulatedPointInFrontOfCameras(point, rotation, position));
}

TEST(TriangulationHelper, CheckEnoughAngle_Sufficient)
{
    constexpr double kMinAngle = 4.;
    constexpr double kAngleBetweenCameras = 5.;
    constexpr int kNumObservations = 50;

    // Try varying numbers of cameras observing a 3d point from 2 to 50 cameras.
    for (auto obs{2}; obs < kNumObservations; ++obs) {
        std::vector<Vector3d> directions;
        directions.reserve(obs);
        for (auto i{0}; i < obs; ++i) {
            const auto rad = math::degToRad(i * kAngleBetweenCameras);
            directions.emplace_back(std::cos(rad), std::sin(rad), 0.);
        }

        EXPECT_TRUE(SufficientTriangulationAngle(directions, kMinAngle));
    }
}

TEST(TriangulationHelper, CheckEnoughAngle_Insufficient)
{
    constexpr double kMinAngle = 4.;
    constexpr int kNumObservations = 50;

    // Try varying numbers of cameras observing a 3d point from 2 to 50 cameras.
    for (auto obs{2}; obs < kNumObservations; ++obs) {
        const auto angle = kMinAngle / (obs + 1e-4);

        std::vector<Vector3d> directions;
        directions.reserve(obs);
        for (auto i{0}; i < obs; ++i) {
            const auto rad = math::degToRad(i * angle);
            directions.emplace_back(std::cos(rad), std::sin(rad), 0.);
        }

        EXPECT_FALSE(SufficientTriangulationAngle(directions, kMinAngle));
    }
}

TEST(TriangulationHelper, CheckEnoughAngle_PartialSufficient)
{
    constexpr double kMinAngle = 4.;

    const std::vector angles{0., 1., 5.};

    std::vector<Vector3d> directions;
    for (const auto& angle : angles) {
        const auto rad = math::degToRad(angle);
        directions.emplace_back(std::cos(rad), std::sin(rad), 0.);
    }

    // 1 out of 2 directions are valid
    EXPECT_TRUE(SufficientTriangulationAngle(directions, kMinAngle));
}

TEST(TriangulationHelper, CheckEnoughAngle_TwoInsufficient)
{
    constexpr double kMinAngle = 4.;

    const std::vector angles{0., 1.};

    std::vector<Vector3d> directions;
    for (const auto& angle : angles) {
        const auto rad = math::degToRad(angle);
        directions.emplace_back(std::cos(rad), std::sin(rad), 0.);
    }

    // 0 out of 1 direction is valid
    EXPECT_FALSE(SufficientTriangulationAngle(directions, kMinAngle));
}
