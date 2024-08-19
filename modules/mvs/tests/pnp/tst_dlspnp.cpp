#include <gtest/gtest.h>

#include <tCore/ContainerUtils>
#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tMath/Eigen/Types>
#include <tMvs/PnP/DlsPnP>

#include "../test_utils.h"

using namespace tl;

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {
constexpr auto kFocalLength{500.};

RandomNumberGenerator kRNG{59};
} // namespace

class DLSPnPTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        //
        _opts.imgNoise.reset();
    }

    void TearDown() override {}

    void Execute(const std::vector<Eigen::Vector3d>& objectPoints,
                 const Eigen::Quaterniond& rotation,
                 const Eigen::Vector3d& translation)
    {
        // Prepare data
        Matrix34d P;
        P << rotation.toRotationMatrix(), translation;

        std::vector<Vector2d> imagePoints;
        imagePoints.reserve(objectPoints.size());
        for (const auto& objPoint : objectPoints) {
            imagePoints.push_back((P * objPoint.homogeneous()).hnormalized());
        }

        if (_opts.imgNoise.has_value()) {
            for (auto& imgPoint : imagePoints) {
                AddNoiseToVector2(_opts.imgNoise.value(), &imgPoint);
            }
        }

        // Run DLS PnP.
        std::vector<Quaterniond> rotations_est;
        std::vector<Vector3d> translations_est;
        EXPECT_TRUE(
            DLSPnp(imagePoints, objectPoints, rotations_est, translations_est));

        // Check at least one of the solutions is correct
        const auto numSolutions = rotations_est.size();

        bool valid{false};
        for (size_t i{0}; i < numSolutions; ++i) {
            Matrix34d P_est;
            P_est << rotations_est[i].toRotationMatrix(), translations_est[i];

            for (size_t j{0}; j < objectPoints.size(); ++j) {
                const Vector2d reproj =
                    (P_est * objectPoints[j].homogeneous()).hnormalized();
                const double rpe = (imagePoints[j] - reproj).squaredNorm();
                ASSERT_LE(rpe, _ref.maxRpe);
            }

            // Check if the solution is accurate.
            const double rotationDiff =
                rotation.angularDistance(rotations_est[i]);
            const double translationDiff =
                (translation - translations_est[i]).squaredNorm();

            if ((rotationDiff < _ref.maxRotationDiff) &&
                (translationDiff < _ref.maxTranslationDiff)) {
                valid = true;
                break;
            }
        }
        EXPECT_TRUE(valid);
    }

protected:
    struct
    {
        std::optional<double> imgNoise = {}; // px
    } _opts;

    struct
    {
        double maxRpe = 1.;          // px^2
        double maxRotationDiff = 1.; // rad
        double maxTranslationDiff = 1.;
    } _ref;
};

TEST_F(DLSPnPTest, MinimumNoNoise)
{
    // Configs
    _ref.maxRpe = 1e-4;
    _ref.maxRotationDiff = 1e-5;
    _ref.maxTranslationDiff = 1e-8;

    // Data
    const std::vector objectPoints{Vector3d{-1., 3., 3.}, Vector3d{1., -1., 2.},
                                   Vector3d{-1., 1., 2.}, Vector3d{2., 1., 3.}};

    const Quaterniond rotation{
        AngleAxisd{math::degToRad(13.), Vector3d::UnitZ()}};
    const Vector3d translation{1., 1., 1.};

    Execute(objectPoints, rotation, translation);
}

TEST_F(DLSPnPTest, SmallScaleWithNoise)
{
    // Configs
    _opts.imgNoise = 1. / kFocalLength;
    _ref.maxRpe = 5e-3;
    _ref.maxRotationDiff = math::degToRad(0.25);
    _ref.maxTranslationDiff = 1e-2;

    // Data
    const std::vector objectPoints{
        Vector3d{-1., 3., 3.}, Vector3d{1., -1., 2.},  Vector3d{-1., 1., 2.},
        Vector3d{2., 1., 3.},  Vector3d{-1., -3., 2.}, Vector3d{1., -2., 1.},
        Vector3d{-1., 4., 2.}, Vector3d{-2., 2., 3.}};

    const Quaterniond rotation{
        AngleAxisd{math::degToRad(13.), Vector3d::UnitZ()}};
    const Vector3d translation{1., 1., 1.};

    Execute(objectPoints, rotation, translation);
}

TEST_F(DLSPnPTest, LargeScaleWithNoise)
{
    // Configs
    _opts.imgNoise = 1. / kFocalLength;
    _ref.maxRpe = 1e-2; // 0.1px
    _ref.maxRotationDiff = math::degToRad(0.3);
    _ref.maxTranslationDiff = 5e-3;

    // Data
    const std::vector<Vector3d> kAxes{Vector3d::UnitZ(),
                                      Vector3d::UnitY(),
                                      Vector3d::UnitX(),
                                      Vector3d{1., 0., 1.}.normalized(),
                                      Vector3d{0., 1., 1.}.normalized(),
                                      Vector3d{1., 1., 1.}.normalized(),
                                      Vector3d{0., 1., 1.}.normalized(),
                                      Vector3d{1., 1., 1.}.normalized()};

    constexpr double kAngles[]{
        math::degToRad(7.),  math::degToRad(12.), math::degToRad(15.),
        math::degToRad(20.), math::degToRad(11.),
        math::degToRad(0.), // Tests no rotation.
        math::degToRad(5.),
        math::degToRad(0.) // Tests no rotation and no translation.
    };

    const std::vector<Vector3d> kTranslations{
        Vector3d{1., 1., 1.},  Vector3d{3., 2., 13.},  Vector3d{4., 5., 11.},
        Vector3d{1., 2., 15.}, Vector3d{3., 1.5, 18.}, Vector3d{1., 7., 11.},
        Vector3d::Zero(), // Tests no translation.
        Vector3d::Zero()  // Tests no translation and no rotation.
    };

    EXPECT_GE(kAxes.size(), std::size(kAngles));
    EXPECT_EQ(kAxes.size(), kTranslations.size());

    constexpr size_t numPoints[]{100, 500, 1000};

    for (size_t i{0}; i < kAxes.size(); ++i) {
        const Quaterniond rotation{AngleAxisd{kAngles[i], kAxes[i]}};
        const auto& translation = kTranslations[i];
        for (const auto& pointCount : numPoints) {
            // Data
            std::vector<Vector3d> objectPoints;
            objectPoints.reserve(pointCount);
            for (size_t k{0}; k < pointCount; ++k) {
                objectPoints.push_back(Vector3d{kRNG.randFloat(-5., 5.),
                                                kRNG.randFloat(-5., 5.),
                                                kRNG.randFloat(2., 10.)});
            }

            Execute(objectPoints, rotation, translation);
        }
    }
}

TEST_F(DLSPnPTest, SmallScaleWithNoise_NoRotation)
{
    // Configs
    _opts.imgNoise = 1. / kFocalLength;
    _ref.maxRpe = 5e-3;
    _ref.maxRotationDiff = math::degToRad(0.25);
    _ref.maxTranslationDiff = 5e-4;

    // Data
    const std::vector objectPoints{
        Vector3d{-1., 3., 3.}, Vector3d{1., -1., 2.},  Vector3d{-1., 1., 2.},
        Vector3d{2., 1., 3.},  Vector3d{-1., -3., 2.}, Vector3d{1., -2., 1.},
        Vector3d{-1., 4., 2.}, Vector3d{-2., 2., 3.}};

    const Quaterniond rotation{
        AngleAxisd{math::degToRad(0.), Vector3d::UnitZ()}};
    const Vector3d translation{1., 1., 1.};

    Execute(objectPoints, rotation, translation);
}

TEST_F(DLSPnPTest, SmallScaleWithNoise_NoTranslation)
{
    // Configs
    _opts.imgNoise = 1. / kFocalLength;
    _ref.maxRpe = 1e-2;
    _ref.maxRotationDiff = math::degToRad(0.2);
    _ref.maxTranslationDiff = 5e-3;

    // Data
    const std::vector objectPoints{
        Vector3d{-1., 3., 3.}, Vector3d{1., -1., 2.},  Vector3d{-1., 1., 2.},
        Vector3d{2., 1., 3.},  Vector3d{-1., -3., 2.}, Vector3d{1., -2., 1.},
        Vector3d{-1., 4., 2.}, Vector3d{-2., 2., 3.}};

    const Quaterniond rotation{
        AngleAxisd{math::degToRad(13.), Vector3d::UnitZ()}};
    const Vector3d translation = Vector3d::Zero();

    Execute(objectPoints, rotation, translation);
}

TEST_F(DLSPnPTest, SmallScaleWithNoise_OrthogonalRotation)
{
    // Configs
    _opts.imgNoise = 1. / kFocalLength;
    _ref.maxRpe = 5e-3;
    _ref.maxRotationDiff = math::degToRad(0.25);
    _ref.maxTranslationDiff = 5e-3;

    // Data
    const std::vector objectPoints{
        Vector3d{-1., 3., 3.}, Vector3d{1., -1., 2.},  Vector3d{-1., 1., 2.},
        Vector3d{2., 1., 3.},  Vector3d{-1., -3., 2.}, Vector3d{1., -2., 1.},
        Vector3d{-1., 4., 2.}, Vector3d{-2., 2., 3.}};

    const Quaterniond rotation{
        AngleAxisd{math::degToRad(90.), Vector3d::UnitZ()}};
    const Vector3d translation{1., 1., 1.};

    Execute(objectPoints, rotation, translation);
}
