#include <algorithm>
#include <vector>

#include <Eigen/Geometry>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tMvs/Feature>
#include <tMvs/Epipolar/Basics>
#include <tMvs/Epipolar/FindFundamental>
#include <tMvs/Epipolar/FindFundamentalEightPoints>
#include <tMvs/Epipolar/Triangulation>

#include "../test_utils.h"

using namespace tl;

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace {
constexpr int kNumTrials = 100;

RandomNumberGenerator kRNG{80};
} // namespace

class Fundamental : public ::testing::Test
{
protected:
    void SetUp() override
    {
        _opts.ransacParams.rng = std::make_shared<RandomNumberGenerator>(kRNG);
        _opts.ransacParams.use_mle = true;
        _opts.ransacParams.error_thresh = 2;
        _opts.ransacParams.failure_probability = 1e-3;
        _opts.noise.reset();
    }

    void TearDown() override {}

    void TestEightPoints(const std::vector<Eigen::Vector3d>& objPoints,
                         const Eigen::Quaterniond& rotation,
                         const Eigen::Vector3d& translation) const
    {
        // Prepare data
        std::vector<Vector2d> imgPoints1;
        std::vector<Vector2d> imgPoints2;
        GenerateImagePoints(objPoints, rotation, translation, &imgPoints1,
                            &imgPoints2);

        // Compute fundamental matrix.
        Matrix3d F;
        EXPECT_TRUE(FindFundamentalEightPoints(imgPoints1, imgPoints2, &F));

        CheckReprojectionError(imgPoints1, imgPoints2, F);
    }

    void TestFindFundamental(const Eigen::Matrix3d& rotation,
                             const Eigen::Vector3d& position) const
    {
        constexpr int kNumCorrespondences = 200;
        const Vector3d translation = (-rotation * position).normalized();

        // Create feature correspondences (inliers and outliers) and add noise
        // if appropriate.
        std::vector<Feature2D2D> matches;
        for (int i = 0; i < kNumCorrespondences; i++) {
            Feature2D2D match;
            if (i < _opts.inlierRatio * kNumCorrespondences) {
                const Vector3d objPoints =
                    Vector3d::Random() + Vector3d{0., 0., 4.};
                match.feature1.pos = _opts.f1 * objPoints.hnormalized();
                match.feature2.pos =
                    _opts.f2 *
                    (rotation * objPoints + translation).hnormalized();

                if (_opts.noise.has_value()) {
                    const auto noise = _opts.noise.value();
                    AddNoiseToVector2(noise, &match.feature1.pos);
                    AddNoiseToVector2(noise, &match.feature2.pos);
                }
            }
            else {
                match.feature1.pos = _opts.f1 * Vector2d::Random();
                match.feature2.pos = _opts.f2 * Vector2d::Random();
            }
            matches.emplace_back(match);
        }

        // Estimate the relative pose.
        UncalibratedRelativePose res;
        SacSummary ransacSummary;
        EXPECT_TRUE(FindFundamental(matches, 600., 2000., RansacType::RANSAC,
                                    _opts.ransacParams, &res, &ransacSummary));

        // Expect that the inlier ratio is close to the ground truth.
        const auto inlierRatio =
            ransacSummary.inliers.size() / static_cast<double>(matches.size());
        EXPECT_GT(inlierRatio, 0.7 * _opts.inlierRatio);

        EXPECT_LT(math::degToRad(
                      AngleAxisd{rotation * res.rotation.transpose()}.angle()),
                  _ref.maxAngleDiff);
        EXPECT_LT(math::degToRad(std::acos(std::clamp(
                      position.normalized().dot(res.position), -1., 1.))),
                  _ref.maxAngleDiff);
    }

    // Creates a test scenario from ground truth 3D points and ground truth
    // rotation and translation. Projection (i.e., image) noise is optional (set
    // to 0 for no noise). The fundamental matrix is computed to ensure that the
    // reprojection errors are sufficiently small.
    void GenerateImagePoints(const std::vector<Eigen::Vector3d>& objPoints,
                             const Eigen::Quaterniond& rotation,
                             const Eigen::Vector3d& translation,
                             std::vector<Eigen::Vector2d>* imgPoints1,
                             std::vector<Eigen::Vector2d>* imgPoints2) const
    {
        const auto numPoints = objPoints.size();

        imgPoints1->reserve(numPoints);
        imgPoints2->reserve(numPoints);
        for (const auto& objPoint : objPoints) {
            imgPoints1->push_back(objPoint.hnormalized());
            imgPoints2->push_back(
                (rotation * objPoint + translation).hnormalized());
        }

        if (_opts.noise.has_value()) {
            const auto& noise = _opts.noise.value();
            for (size_t i{0}; i < numPoints; ++i) {
                AddNoiseToVector2(noise, &((*imgPoints1)[i]));
                AddNoiseToVector2(noise, &((*imgPoints2)[i]));
            }
        }
    }

protected:
    struct
    {
        SacParameters ransacParams = {};
        double f1, f2;
        double inlierRatio{1.};
        std::optional<double> noise;
    } _opts;

    struct
    {
        double maxAngleDiff{0.};
        double maxRpe{0.};
    } _ref;

private:
    void CheckReprojectionError(const std::vector<Eigen::Vector2d>& imgPoints1,
                                const std::vector<Eigen::Vector2d>& imgPoints2,
                                const Eigen::Matrix3d& F) const
    {
        // Compute the projection matrices.
        Matrix34d P_left, P_right;
        ProjectionMatricesFromFundamentalMatrix(F.data(), P_right.data(),
                                                P_left.data());

        for (int i = 0; i < imgPoints1.size(); i++) {
            // Triangulate the world point.
            Vector4d objPoint;
            CHECK(TriangulateDLT(P_left, P_right, imgPoints1[i], imgPoints2[i],
                                 &objPoint));
            const Vector3d objPoint_left = P_left * objPoint;
            const Vector3d objPoint_right = P_right * objPoint;

            // Compute reprojection error.
            const double error1 =
                (imgPoints1[i] - objPoint_left.hnormalized()).squaredNorm();
            const double error2 =
                (imgPoints2[i] - objPoint_right.hnormalized()).squaredNorm();

            EXPECT_LT(error1, _ref.maxRpe);
            EXPECT_LT(error2, _ref.maxRpe);
        }
    }
};

TEST_F(Fundamental, EightPointsNoNoise)
{
    _ref.maxRpe = 1e-12;

    const std::vector objPoints{Vector3d{-1., 3., 3.},  Vector3d{1., -1., 2.},
                                Vector3d{-1., 1., 2.},  Vector3d{2., 1., 3.},
                                Vector3d{-1., -3., 2.}, Vector3d{1., -2., 1.},
                                Vector3d{-1., 4., 2.},  Vector3d{-2., 2., 3.}};

    const Quaterniond rotation{
        AngleAxisd{math::radToDeg(13.), Vector3d::UnitZ()}};
    const Vector3d translation{1., 0.5, 1.5};

    TestEightPoints(objPoints, rotation, translation);
}

TEST_F(Fundamental, EightPointsWithNoise)
{
    _opts.noise = 1. / 512.;
    _ref.maxRpe = 1e-4;

    const std::vector objPoints{Vector3d{-1., 3., 3.},  Vector3d{1., -1., 2.},
                                Vector3d{-1., 1., 2.},  Vector3d{2., 1., 3.},
                                Vector3d{-1., -3., 2.}, Vector3d{1., -2., 1.},
                                Vector3d{-1., 4., 2.},  Vector3d{-2., 2., 3.}};

    const Quaterniond rotation{
        AngleAxisd{math::radToDeg(13.), Vector3d::UnitZ()}};
    const Vector3d translation{1., 0.5, 0.};

    TestEightPoints(objPoints, rotation, translation);
}

TEST_F(Fundamental, ManyPointsWithNoise_Overconstrained)
{
    _opts.noise = 1. / 512.;
    _ref.maxRpe = 1e-4;

    const std::vector objPoints{
        Vector3d{-1., 3., 3.}, Vector3d{1., -1., 2.},  Vector3d{-1., 1., 2.},
        Vector3d{2., 1., 3.},  Vector3d{-1., -3., 2.}, Vector3d{1., -2., 1.},
        Vector3d{-1., 4., 2.}, Vector3d{-2., 2., 3.},  Vector3d{-2., 4., 1.},
        Vector3d{-2., 2., 2.}, Vector3d{-4., 3., 3.}};

    const Quaterniond rotation{
        AngleAxisd{math::radToDeg(13.), Vector3d::UnitZ()}};
    const Vector3d translation{1., 0.5, 0.};

    TestEightPoints(objPoints, rotation, translation);
}

TEST_F(Fundamental, ManyPointsNoNoise_Degenerate)
{
    const std::vector objPoints{Vector3d{-1., 3., 3.},  Vector3d{-1., 3., 3.},
                                Vector3d{-1., 1., 2.},  Vector3d{2., 1., 3.},
                                Vector3d{-1., -3., 2.}, Vector3d{1., -2., 1.},
                                Vector3d{-1., 4., 2.},  Vector3d{-2., 2., 3.}};

    const Quaterniond rotation{
        AngleAxisd{math::radToDeg(13.), Vector3d::UnitZ()}};
    const Vector3d translation{1., 0.5, 0.};

    std::vector<Vector2d> imgPoints1;
    std::vector<Vector2d> imgPoints2;
    GenerateImagePoints(objPoints, rotation, translation, &imgPoints1,
                        &imgPoints2);

    Matrix3d F;
    EXPECT_FALSE(FindFundamentalEightPoints(imgPoints1, imgPoints2, &F));
}

TEST_F(Fundamental, FindFundamental_AllInliersNoNoise)
{
    _opts.inlierRatio = 1.;
    _ref.maxAngleDiff = 1e-4;

    for (int i{0}; i < kNumTrials; ++i) {
        _opts.f1 = kRNG.randFloat(800., 1600.);
        _opts.f2 = kRNG.randFloat(800., 1600.);

        const Matrix3d rotation = RandomRotation(10.);
        const Vector3d position = Vector3d::Random();
        TestFindFundamental(rotation, position);
    }
}

// FIXME: Seriously fail
TEST_F(Fundamental, FindFundamental_AllInliersWithNoise)
{
    _opts.inlierRatio = 1.;
    _opts.noise = 1.;
    _ref.maxAngleDiff = 20.;

    for (int k = 0; k < kNumTrials; k++) {
        _opts.f1 = kRNG.randFloat(800., 1600.);
        _opts.f2 = kRNG.randFloat(800., 1600.);

        const Matrix3d rotation = RandomRotation(10.);
        const Vector3d position = Vector3d::Random();
        TestFindFundamental(rotation, position);
    }
}

TEST_F(Fundamental, FindFundamental_OutliersNoNoise)
{
    _opts.inlierRatio = 0.7;
    _ref.maxAngleDiff = 20.;

    for (int k = 0; k < kNumTrials; k++) {
        _opts.f1 = kRNG.randFloat(800., 1600.);
        _opts.f2 = kRNG.randFloat(800., 1600.);

        const Matrix3d rotation = RandomRotation(10.);
        const Vector3d position = Vector3d::Random();
        TestFindFundamental(rotation, position);
    }
}

TEST_F(Fundamental, FindFundamental_OutliersWithNoise)
{
    _opts.ransacParams.error_thresh = 4. * 4.;
    _opts.ransacParams.max_iterations = 1000;
    _opts.inlierRatio = 0.7;
    _opts.noise = 1.;
    _ref.maxAngleDiff = 20.;

    for (int k = 0; k < kNumTrials; k++) {
        _opts.f1 = kRNG.randFloat(800., 1600.);
        _opts.f2 = kRNG.randFloat(800., 1600.);

        const Matrix3d rotation = RandomRotation(10.);
        const Vector3d position = Vector3d::Random();
        TestFindFundamental(rotation, position);
    }
}

TEST_F(Fundamental, FindFundamental_OutliersWithNoise_LO)
{
    _opts.ransacParams.error_thresh = 4. * 4.;
    _opts.ransacParams.max_iterations = 1000;
    _opts.ransacParams.use_lo = true;
    _opts.ransacParams.lo_start_iterations = 10;
    _opts.inlierRatio = 0.7;
    _opts.noise = 1.;
    _ref.maxAngleDiff = 20.;

    for (int k = 0; k < kNumTrials; k++) {
        _opts.f1 = kRNG.randFloat(800., 1600.);
        _opts.f2 = kRNG.randFloat(800., 1600.);

        const Matrix3d rotation = RandomRotation(10.);
        const Vector3d position = Vector3d::Random();
        TestFindFundamental(rotation, position);
    }
}

TEST(FundamentalOpenCV, FindFundamental)
{
    //
    GTEST_SKIP() << "Not implement...";
}
