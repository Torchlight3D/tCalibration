#include <Eigen/Geometry>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tMath/Eigen/Utils>
#include <tMvs/Feature>
#include <tMvs/Epipolar/Basics>
#include <tMvs/Epipolar/FindEssential>
#include <tMvs/Epipolar/FindEssentialFivePoints>

#include "../test_utils.h"

using namespace tl;

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {
constexpr double kFocalLength = 1000.;
constexpr double kSampsonError = 2.;
constexpr double kErrorThreshold =
    (kSampsonError * kSampsonError) / (kFocalLength * kFocalLength);

RandomNumberGenerator kRNG{65};
} // namespace

class Essential : public ::testing::Test
{
protected:
    void SetUp() override
    {
        _opts.ransacParams.rng = std::make_shared<RandomNumberGenerator>(kRNG);
        _opts.ransacParams.use_mle = true;
        _opts.ransacParams.error_thresh = kErrorThreshold;
        _opts.ransacParams.failure_probability = 1e-4;
        _opts.noise.reset();
    }

    void TearDown() override {}

    void TestFivePoints(const std::vector<Eigen::Vector3d>& objPoints,
                        const Eigen::Matrix3d& rotation,
                        const Eigen::Vector3d& translation) const
    {
        const auto numPoints = objPoints.size();

        // Prepare data
        std::vector<Vector2d> imgPoints1;
        std::vector<Vector2d> imgPoints2;
        for (const auto& objPoint : objPoints) {
            const Vector3d proj = rotation * objPoint + translation;
            imgPoints1.push_back(objPoint.hnormalized());
            imgPoints2.push_back(proj.hnormalized());
        }

        if (_opts.noise.has_value()) {
            for (size_t i{0}; i < numPoints; ++i) {
                AddNoiseToVector2(_opts.noise.value(), &imgPoints1[i]);
                AddNoiseToVector2(_opts.noise.value(), &imgPoints2[i]);
            }
        }

        const Matrix3d E = math::Skew(translation) * rotation;

        // Calculates the essential matrix, this may return multiple solutions.
        std::vector<Matrix3d> ematrices;
        EXPECT_TRUE(
            FindEssentialFivePoints(imgPoints1, imgPoints2, &ematrices));

        // At least one of the solutions is correct
        const auto valid =
            std::any_of(ematrices.cbegin(), ematrices.cend(),
                        [&E, tol = _ref.E_tolerance](const auto& E_est) {
                            return math::ArraysEqualUpToScale(9, E_est.data(),
                                                              E.data(), tol);
                            // return E_est.isApprox(E, tol);
                        });
        EXPECT_TRUE(valid);

        // All solutions should have valid epipolar constraints.
        const double kEpipolarTolerance = objPoints.size() == 5 ? 1e-8 : 1e-3;
        for (const auto& E_est : ematrices) {
            for (size_t i{0}; i < numPoints; ++i) {
                EXPECT_LT(
                    SquaredSampsonDistance(E_est, imgPoints1[i], imgPoints2[i]),
                    kEpipolarTolerance);
            }
        }
    }

    void TestFindEssential(const Eigen::Matrix3d& rotation,
                           const Eigen::Vector3d& position) const
    {
        // Prepare data
        const auto objPoints = GeneratePoints();
        const auto numPoints = objPoints.size();
        const Vector3d translation = (-rotation * position).normalized();

        std::vector<Feature2D2D> corrs;
        for (size_t i{0}; i < numPoints; ++i) {
            Feature2D2D corr;
            if (i < _opts.inlierRatio * numPoints) {
                // Make sure the point is in front of the camera.
                corr.feature1.pos = objPoints[i].hnormalized();
                corr.feature2.pos =
                    (rotation * objPoints[i] + translation).hnormalized();
            }
            else {
                corr.feature1.pos = Vector2d::Random();
                corr.feature2.pos = Vector2d::Random();
            }
            corrs.push_back(corr);
        }

        if (_opts.noise.has_value()) {
            const auto noise = _opts.noise.value() / kFocalLength;
            for (int i = 0; i < objPoints.size(); i++) {
                AddNoiseToVector2(noise, &corrs[i].feature1.pos);
                AddNoiseToVector2(noise, &corrs[i].feature2.pos);
            }
        }

        // Estimate the relative pose.
        RelativePose pose;
        SacSummary ransacSummary;
        EXPECT_TRUE(EstimateRelativePose(_opts.ransacParams, RansacType::RANSAC,
                                         corrs, &pose, &ransacSummary));

        // Expect that the inlier ratio is close to the ground truth.
        EXPECT_GT(static_cast<double>(ransacSummary.inliers.size()), 5);

        LOG(INFO) << "Num lo-ransac iterations: "
                  << ransacSummary.num_lo_iterations;

        AngleAxisd rotation_loop{rotation * pose.rotation.transpose()};
        EXPECT_LT(math::radToDeg(rotation_loop.angle()), _ref.maxAngleDiff);

        const double translation_diff_rad = std::acos(
            std::clamp(position.normalized().dot(pose.position), -1., 1.));

        EXPECT_LT(math::radToDeg(translation_diff_rad), _ref.maxAngleDiff);
    }

protected:
    struct
    {
        SacParameters ransacParams = {};
        std::optional<double> noise;
        double inlierRatio = 0.;
    } _opts;

    struct
    {
        double maxAngleDiff = 0.; // deg
        double maxDiff = 0.;
        double E_tolerance;
    } _ref;

private:
    // Generate points in a grid so that they are repeatable.
    static std::vector<Vector3d> GeneratePoints()
    {
        std::vector<Vector3d> points;
        for (int x = -1; x <= 1; x++) {
            for (int y = -1; y <= 1; y++) {
                for (int depth = 4; depth <= 6; depth++) {
                    points.push_back(Vector3d(x, y, depth));
                }
            }
        }

        return points;
    }
};

TEST_F(Essential, FivePointsNoNoise)
{
    _ref.E_tolerance = 1e-4;

    const std::vector objPoints{Vector3d{-1., 3., 3.}, Vector3d{1., -1., 2.},
                                Vector3d{3., 1., 2.5}, Vector3d{-1., 1., 2.},
                                Vector3d{2., 1., 3.}};
    const Matrix3d rotation =
        AngleAxisd{math::degToRad(13.), Vector3d::UnitZ()}.toRotationMatrix();
    const Vector3d translation{1., 1., 1.};

    TestFivePoints(objPoints, rotation, translation);
}

TEST_F(Essential, FivePointsWithNoise)
{
    _opts.noise = 1. / 512.;
    _ref.E_tolerance = 1e-2;

    const std::vector objPoints{Vector3d{-1., 3., 3.}, Vector3d{1., -1., 2.},
                                Vector3d{3., 1., 2.5}, Vector3d{-1., 1., 2.},
                                Vector3d{2., 1., 3.}};
    const Matrix3d rotation =
        AngleAxisd{math::degToRad(13.), Vector3d::UnitZ()}.toRotationMatrix();
    const Vector3d translation{1., 1., 1.};

    TestFivePoints(objPoints, rotation, translation);
}

TEST_F(Essential, FivePointsWithNoise_ForwardMotion)
{
    _opts.noise = 1. / 512.;
    _ref.E_tolerance = 0.15;

    const std::vector objPoints{Vector3d{-1., 3., 3.}, Vector3d{1., -1., 2.},
                                Vector3d{3., 1., 2.}, Vector3d{-1., 1., 2.},
                                Vector3d{2., 1., 3.}};
    const Matrix3d rotation =
        AngleAxisd{math::degToRad(13.), Vector3d::UnitZ()}.toRotationMatrix();
    const Vector3d translation{0., 0., 1.};

    TestFivePoints(objPoints, rotation, translation);
}

TEST_F(Essential, FivePointsWithNoise_NoRotation)
{
    _opts.noise = 1. / 512.;
    _ref.E_tolerance = 1e-2;

    const std::vector objPoints{Vector3d{-1., 3., 3.}, Vector3d{1., -1., 2.},
                                Vector3d{3., 1., 2.}, Vector3d{-1., 1., 2.},
                                Vector3d{2., 1., 3.}};
    const Matrix3d rotation = Matrix3d::Identity();
    const Vector3d translation{1., 1., 1.};

    TestFivePoints(objPoints, rotation, translation);
}

TEST_F(Essential, ManyPointsNoNoise)
{
    _ref.E_tolerance = 1e-4;

    const std::vector objPoints{Vector3d{-1., 3., 3.}, Vector3d{1., -1., 2.},
                                Vector3d{3., 1., 2.5}, Vector3d{-1., 1., 2.},
                                Vector3d{2., 1., 3.},  Vector3d{1.7, 2.1, 3.2}};
    const Matrix3d rotation =
        AngleAxisd{math::degToRad(13.), Vector3d::UnitZ()}.toRotationMatrix();
    const Vector3d translation{1., 1., 1.};

    TestFivePoints(objPoints, rotation, translation);
}

TEST_F(Essential, ManyPointsWithNoise)
{
    _opts.noise = 1. / 512.;
    _ref.E_tolerance = 1e-2;

    const std::vector objPoints{Vector3d{-1., 3., 3.}, Vector3d{1., -1., 2.},
                                Vector3d{3., 1., 2.5}, Vector3d{-1., 1., 2.},
                                Vector3d{2., 1., 3.},  Vector3d{1.7, 2.1, 3.2}};
    const Matrix3d rotation =
        AngleAxisd{math::degToRad(13.), Vector3d::UnitZ()}.toRotationMatrix();
    const Vector3d translation{1., 1., 1.};

    TestFivePoints(objPoints, rotation, translation);
}

TEST_F(Essential, ManyPointsWithNoise_ForwardMotion)
{
    _opts.noise = 1. / 512.;
    _ref.E_tolerance = 0.15;

    const std::vector objPoints{Vector3d{-1., 3., 3.}, Vector3d{1., -1., 2.},
                                Vector3d{3., 1., 2.},  Vector3d{-1., 1., 2.},
                                Vector3d{2., 1., 3.},  Vector3d{1.7, 2.1, 3.2}};
    const Matrix3d rotation =
        AngleAxisd{math::degToRad(13.), Vector3d::UnitZ()}.toRotationMatrix();
    const Vector3d translation{0., 0., 1.};

    TestFivePoints(objPoints, rotation, translation);
}

TEST_F(Essential, ManyPointsWithNoise_NoRotation)
{
    _opts.noise = 1. / 512.;
    _ref.E_tolerance = 1e-2;

    const std::vector objPoints{Vector3d{-1., 3., 3.}, Vector3d{1., -1., 2.},
                                Vector3d{3., 1., 2.},  Vector3d{-1., 1., 2.},
                                Vector3d{2., 1., 3.},  Vector3d{1.7, 2.1, 3.2}};
    const Matrix3d rotation = Matrix3d::Identity();
    const Vector3d translation{1., 1., 1.};

    TestFivePoints(objPoints, rotation, translation);
}

TEST_F(Essential, FindEssential_AllInliersNoNoise)
{
    _opts.inlierRatio = 1.;
    _ref.maxAngleDiff = 1e-4;

    const std::vector<Matrix3d> rotations{
        AngleAxisd{math::degToRad(12.), Vector3d::UnitY()}.toRotationMatrix(),
        AngleAxisd{math::degToRad(-9.), Vector3d{1., 0.2, -0.8}.normalized()}
            .toRotationMatrix()};
    const std::vector positions{Vector3d{-0.7, 0., 0.}, Vector3d{0., 0.1, 0.5}};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            TestFindEssential(rotation, position);
        }
    }
}

TEST_F(Essential, FindEssential_AllInliersWithNoise)
{
    _opts.inlierRatio = 1.;
    _opts.noise = 1.;
    _ref.maxAngleDiff = 5.;

    const std::vector<Matrix3d> rotations{
        Matrix3d::Identity(),
        AngleAxisd(math::degToRad(12.), Vector3d::UnitY()).toRotationMatrix(),
        AngleAxisd(math::degToRad(-9.), Vector3d{1., 0.2, -0.8}.normalized())
            .toRotationMatrix()};
    const std::vector positions{Vector3d{-1.3, 0., 0.}, Vector3d{0., 0.1, 0.5}};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            TestFindEssential(rotation, position);
        }
    }
}

TEST_F(Essential, FindEssential_OutliersNoNoise)
{
    _opts.inlierRatio = 0.7;
    _ref.maxAngleDiff = 5.;

    const std::vector<Matrix3d> rotations{Matrix3d::Identity(),
                                          RandomRotation(10.)};
    const std::vector positions{Vector3d{1., 0.2, 0.}, Vector3d{0., 1., 0.1}};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            TestFindEssential(rotation, position);
        }
    }
}

TEST_F(Essential, FindEssential_OutliersWithNoise)
{
    _opts.ransacParams.failure_probability = 1e-3;
    _opts.inlierRatio = 0.7;
    _opts.noise = 1.;
    _ref.maxAngleDiff = 5.;

    const std::vector<Matrix3d> rotations{Matrix3d::Identity(),
                                          RandomRotation(10.)};
    const std::vector positions{Vector3d{1., 0.2, 0.}, Vector3d{0., 1., 0.1}};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            TestFindEssential(rotation, position);
        }
    }
}

TEST_F(Essential, FindEssential_OutliersWithNoise_LO)
{
    _opts.ransacParams.failure_probability = 1e-3;
    _opts.ransacParams.use_lo = true;
    _opts.ransacParams.lo_start_iterations = 5;
    _opts.inlierRatio = 0.7;
    _opts.noise = 1.;
    _ref.maxAngleDiff = 5.;

    const std::vector<Matrix3d> rotations{Matrix3d::Identity(),
                                          RandomRotation(10.)};
    const std::vector positions{Vector3d{1., 0.2, 0.}, Vector3d{0., 1., 0.1}};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            TestFindEssential(rotation, position);
        }
    }
}

TEST(EssentialOpenCV, FindEssential_)
{
    //
    GTEST_SKIP() << "Not implement...";
}
