#include <Eigen/Geometry>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>

#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tMVS/Feature>
#include <tMVS/Epipolar/FindHomography>
#include <tMVS/Epipolar/FindHomographyFourPoints>

#include "test_utils.h"

using namespace tl;

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {
constexpr double kFocalLength = 1000.;
constexpr double kReprojectionError = 4.;
constexpr double kErrorThreshold =
    (kReprojectionError * kReprojectionError) / (kFocalLength * kFocalLength);

RandomNumberGenerator kRNG{61};
} // namespace

class Homography : public ::testing::Test
{
protected:
    void SetUp() override
    {
        _opts.ransacParams.rng = std::make_shared<RandomNumberGenerator>(kRNG);
        _opts.ransacParams.use_mle = true;
        _opts.ransacParams.error_thresh = kErrorThreshold;
        _opts.ransacParams.failure_probability = 1e-3;
        _opts.noise.reset();
    }

    void TearDown() override {}

    void TestFourPoints(const std::vector<Vector3d>& objPoints,
                        const Quaterniond& rotation,
                        const Vector3d& translation) const
    {
        std::vector<Vector2d> imgPoints1;
        std::vector<Vector2d> imgPoints2;
        GenerateImagePoints(objPoints, rotation, translation, &imgPoints1,
                            &imgPoints2);

        Matrix3d H;
        EXPECT_TRUE(FourPointHomography(imgPoints1, imgPoints2, &H));

        CheckSymmetricError(imgPoints1, imgPoints2, H);
    }

    void TestFindHomography(const Eigen::Matrix3d& orientation,
                            const Eigen::Vector3d& position) const
    {
        // Prepare data
        auto objPoints = GeneratePoints();
        const auto numPoints = objPoints.size();
        const Vector3d translation = (-orientation * position).normalized();

        std::vector<Feature2D2D> corrs;
        for (size_t i{0}; i < numPoints; ++i) {
            Feature2D2D corr;
            if (i < _opts.inlierRatio * numPoints) {
                corr.feature1.pos = objPoints[i].hnormalized();
                corr.feature2.pos =
                    (orientation * objPoints[i] + translation).hnormalized();
            }
            else {
                corr.feature1.pos = Vector2d::Random();
                corr.feature2.pos = Vector2d::Random();
            }
            corrs.push_back(corr);
        }

        if (_opts.noise.has_value()) {
            const auto noise = _opts.noise.value() / kFocalLength;
            for (auto& corr : corrs) {
                AddNoiseToVector2(noise, &corr.feature1.pos);
                AddNoiseToVector2(noise, &corr.feature2.pos);
            }
        }

        // Estimate the H
        Matrix3d H;
        SacSummary ransacSummary;
        EXPECT_TRUE(FindHomography(corrs, RansacType::RANSAC,
                                   _opts.ransacParams, &H, &ransacSummary));

        // Expect that the inlier ratio is close to the ground truth.
        EXPECT_GT(static_cast<double>(ransacSummary.inliers.size()), 4);
    }

    void TestFindHomographyOpenCV(const Eigen::Matrix3d& orientation,
                                  const Eigen::Vector3d& position) const
    {
        // Prepare data
        auto objPoints = GeneratePoints();
        const auto numPoints = objPoints.size();
        const Vector3d translation = (-orientation * position).normalized();

        std::vector<cv::Point2f> imgPoints1, imgPoints2;
        for (size_t i{0}; i < numPoints; ++i) {
            Vector2d ipoint1, ipoint2;
            if (i < _opts.inlierRatio * numPoints) {
                ipoint1 = objPoints[i].hnormalized();
                ipoint2 =
                    (orientation * objPoints[i] + translation).hnormalized();
            }
            else {
                ipoint1 = Vector2d::Random();
                ipoint2 = Vector2d::Random();
            }

            imgPoints1.emplace_back(ipoint1.x(), ipoint1.y());
            imgPoints2.emplace_back(ipoint2.x(), ipoint2.y());
        }

        // if (_opts.noise.has_value()) {
        //     const auto noise = _opts.noise.value() / kFocalLength;
        //     for (auto& corr : corrs) {
        //         AddNoiseToProjection(noise, &kRNG, &corr.feature1.pos);
        //         AddNoiseToProjection(noise, &kRNG, &corr.feature2.pos);
        //     }
        // }

        // // Estimate the H
        // Matrix3d H;
        // SacSummary ransacSummary;
        // EXPECT_TRUE(FindHomography(corrs, RansacType::RANSAC,
        //                            _opts.ransacParams, &H, &ransacSummary));

        // // Expect that the inlier ratio is close to the ground truth.
        // EXPECT_GT(static_cast<double>(ransacSummary.inliers.size()), 4);
    }

protected:
    struct
    {
        SacParameters ransacParams;
        std::optional<double> noise;
        double inlierRatio{1.};
    } _opts;

    struct
    {
        double maxSymmetricError{0.};
        double maxPoseDiff;
    } _ref;

private:
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
            for (size_t i{0}; i < numPoints; ++i) {
                AddNoiseToVector2(_opts.noise.value(), &((*imgPoints1)[i]));
                AddNoiseToVector2(_opts.noise.value(), &((*imgPoints2)[i]));
            }
        }
    }

    std::vector<Eigen::Vector3d> GeneratePoints(double depth = 5.) const
    {
        std::vector<Vector3d> points;
        for (int x = -4; x <= 4; x++) {
            for (int y = -4; y <= 4; y++) {
                points.push_back(Vector3d(x, y, depth));
            }
        }

        return points;
    }

    void CheckSymmetricError(const std::vector<Eigen::Vector2d>& imgPoints1,
                             const std::vector<Eigen::Vector2d>& imgPoints2,
                             const Matrix3d& H) const
    {
        const Matrix3d H_inv = H.inverse();
        for (int i = 0; i < imgPoints1.size(); i++) {
            const Vector3d imgPoint1_hat = H_inv * imgPoints2[i].homogeneous();
            const Vector3d imgPoint2_hat = H * imgPoints1[i].homogeneous();
            const double error1 =
                (imgPoints1[i] - imgPoint1_hat.hnormalized()).squaredNorm();
            const double error2 =
                (imgPoints2[i] - imgPoint2_hat.hnormalized()).squaredNorm();

            EXPECT_LT(error1, _ref.maxSymmetricError);
            EXPECT_LT(error2, _ref.maxSymmetricError);
        }
    }
};

TEST_F(Homography, FourPointsNoNoise)
{
    _ref.maxSymmetricError = 1e-12;

    const std::vector objPoints{Vector3d{-1., 3., 3.}, Vector3d{1., -1., 2.},
                                Vector3d{-1., 1., 2.}, Vector3d{2., 1., 3.}};

    const Quaterniond rotation{
        AngleAxisd{math::degToRad(13.), Vector3d::UnitZ()}};
    const Vector3d translation{0., 0., 0.};

    TestFourPoints(objPoints, rotation, translation);
}

// NOTE: Use same data in Homography.FourPointsNoNoise, but with noise
TEST_F(Homography, FourPointsWithNoise)
{
    _opts.noise = 1. / 512.;
    _ref.maxSymmetricError = 1e-4;

    const std::vector objPoints{Vector3d{-1., 3., 3.}, Vector3d{1., -1., 2.},
                                Vector3d{-1., 1., 2.}, Vector3d{2., 1., 3.}};

    const Quaterniond rotation{
        AngleAxisd{math::degToRad(13.), Vector3d::UnitZ()}};
    const Vector3d translation{0., 0., 0.};

    TestFourPoints(objPoints, rotation, translation);
}

TEST_F(Homography, FourPointsInPlaneWithNoise)
{
    _opts.noise = 1. / 512.;
    _ref.maxSymmetricError = 1e-4;

    const std::vector objPoints{Vector3d{-1., 3., 5.}, Vector3d{1., -1., 5.},
                                Vector3d{-1., 1., 5.}, Vector3d{2., 1., 5.}};

    const Quaterniond rotation{
        AngleAxisd{math::degToRad(13.), Vector3d::UnitZ()}};
    const Vector3d translation{1., -0.5, -1.};

    TestFourPoints(objPoints, rotation, translation);
}

TEST_F(Homography, ManyPointsWithNoise)
{
    _opts.noise = 1. / 512.;
    _ref.maxSymmetricError = 1e-4;

    constexpr int kNumPoints = 100;
    std::vector<Vector3d> objPoints(kNumPoints);
    for (int i{0}; i < kNumPoints; ++i) {
        objPoints[i] = {kRNG.randFloat(-2., 2.), kRNG.randFloat(-2., 2.),
                        kRNG.randFloat(1., 5.)};
    }

    const Quaterniond rotation{
        AngleAxisd{math::degToRad(13.), Vector3d::UnitZ()}};
    const Vector3d translation{0., 0., 0.};

    TestFourPoints(objPoints, rotation, translation);
}

TEST_F(Homography, FindHomography_AllInliersNoNoise)
{
    _opts.inlierRatio = 1.;
    _ref.maxPoseDiff = 1e-4;

    const std::vector<Matrix3d> rotations{
        Matrix3d::Identity(),
        AngleAxisd{math::degToRad(12.), Vector3d::UnitY()}.toRotationMatrix(),
        AngleAxisd{math::degToRad(-9.), Vector3d{1., 0.2, -0.8}.normalized()}
            .toRotationMatrix()};
    const std::vector positions{Vector3d{-1.3, 0., 0.}, Vector3d{0., 0., 0.5}};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            TestFindHomography(rotation, position);
        }
    }
}

TEST_F(Homography, FindHomography_AllInliersWithNoise)
{
    _opts.inlierRatio = 1.;
    _opts.noise = 1.;
    _ref.maxPoseDiff = 1e-2;

    const std::vector<Matrix3d> rotations{
        Matrix3d::Identity(),
        AngleAxisd{math::degToRad(12.), Vector3d::UnitY()}.toRotationMatrix(),
        AngleAxisd{math::degToRad(-9.), Vector3d{1., 0.2, -0.8}.normalized()}
            .toRotationMatrix()};
    const std::vector positions{Vector3d{-1.3, 0., 0.}, Vector3d{0., 0., 0.5}};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            TestFindHomography(rotation, position);
        }
    }
}

TEST_F(Homography, FindHomography_OutliersNoNoise)
{
    _opts.inlierRatio = 0.7;
    _ref.maxPoseDiff = 1e-2;

    const std::vector<Matrix3d> rotations{Matrix3d::Identity(),
                                          RandomRotation(10.)};
    const std::vector positions{Vector3d::UnitX(), Vector3d::UnitY()};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            TestFindHomography(rotation, position);
        }
    }
}

TEST_F(Homography, FindHomography_OutliersWithNoise)
{
    _opts.inlierRatio = 0.7;
    _opts.noise = 1.;
    _ref.maxPoseDiff = 1e-2;

    const std::vector<Matrix3d> rotations{Matrix3d::Identity(),
                                          RandomRotation(10.)};
    const std::vector positions{Vector3d::UnitX(), Vector3d::UnitY()};

    for (const auto& rotation : rotations) {
        for (const auto& position : positions) {
            TestFindHomography(rotation, position);
        }
    }
}

TEST(HomographyOpenCV, FindHomography)
{
    //
    GTEST_SKIP() << "Not Implement...";
}
