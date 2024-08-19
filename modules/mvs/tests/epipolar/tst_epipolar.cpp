#include <algorithm>

#include <Eigen/Geometry>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <tCore/RandomGenerator>
#include <tMath/Eigen/Utils>
#include <tMvs/Epipolar/Basics>
#include <tMvs/Feature>

#include "../test_utils.h"

using namespace tl;

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

namespace {
RandomNumberGenerator kRNG{51};
}

TEST(EpipolarBasics, DecomposeEssentialMatrix)
{
    constexpr double kTranslationTolerance = 1e-6;
    constexpr double kRotationTolerance = 1e-4;
    constexpr auto kSampleCount{100};

    for (auto i{0}; i < kSampleCount; i++) {
        const Matrix3d R = RandomRotation(10.);
        const Vector3d t = Vector3d::Random().normalized();
        const Matrix3d E = math::Skew(t) * R;

        Matrix3d rotation1, rotation2;
        Vector3d t_est;
        DecomposeEssentialMatrix(E, &rotation1, &rotation2, &t_est);

        const double translation_dist =
            std::min((t_est - t).norm(), (t_est + t).norm());

        const AngleAxisd rotation1_aa(R.transpose() * rotation1);
        const AngleAxisd rotation2_aa(R.transpose() * rotation2);
        const double rotation1_dist = rotation1_aa.angle();
        const double rotation2_dist = rotation2_aa.angle();

        EXPECT_TRUE(translation_dist < kTranslationTolerance &&
                    (rotation1_dist < kRotationTolerance ||
                     rotation2_dist < kRotationTolerance));
    }
}

TEST(EpipolarBasics, EssentialMatrixFromTwoProjectionMatrices)
{
    constexpr double kTranslationTolerance = 1e-6;
    constexpr double kRotationTolerance = 1e-4;

    for (int i = 0; i < 100; i++) {
        const Matrix3d in_rotation1 = RandomRotation(10.);
        // Or rng.RandVector3d().normalized();
        const Vector3d in_position1 = Vector3d::Zero();
        const Vector3d in_translation1 = -in_rotation1 * in_position1;
        const Matrix3d in_rotation2 = RandomRotation(10.);
        const Vector3d in_position2 = Vector3d::Random().normalized();
        const Vector3d in_translation2 = -in_rotation2 * in_position2;

        Matrix34d proj1, proj2;
        proj1.leftCols<3>() = in_rotation1;
        proj1.rightCols<1>() = in_translation1;
        proj2.leftCols<3>() = in_rotation2;
        proj2.rightCols<1>() = in_translation2;

        // Get the essential matrix.
        Matrix3d essential_matrix;
        EssentialMatrixFromTwoProjectionMatrices(proj1, proj2,
                                                 &essential_matrix);

        Matrix3d rotation1, rotation2;
        Vector3d translation;
        DecomposeEssentialMatrix(essential_matrix, &rotation1, &rotation2,
                                 &translation);

        const Vector3d gt_translation =
            -in_rotation1 * (in_position2 - in_position1);
        const double translation_dist =
            std::min((translation - gt_translation).norm(),
                     (translation + gt_translation).norm());

        const Matrix3d gt_rotation = in_rotation1 * in_rotation2.transpose();
        const AngleAxisd rotation1_aa(gt_rotation.transpose() * rotation1);
        const AngleAxisd rotation2_aa(gt_rotation.transpose() * rotation2);
        const double rotation1_dist = rotation1_aa.angle();
        const double rotation2_dist = rotation2_aa.angle();

        ASSERT_TRUE(translation_dist < kTranslationTolerance);
        ASSERT_TRUE(rotation1_dist < kRotationTolerance ||
                    rotation2_dist < kRotationTolerance);
    }
}

void TestGetBestPoseFromEssentialMatrix(int num_inliers, int num_outliers)
{
    constexpr double kTolerance = 1e-12;

    for (int i = 0; i < 100; i++) {
        const Matrix3d gt_rotation = RandomRotation(15.);

        const Vector3d gt_translation = Vector3d::Random().normalized();
        const Vector3d gt_position = -gt_rotation.transpose() * gt_translation;
        const Matrix3d ematrix = math::Skew(gt_translation) * gt_rotation;

        // Create Correspondences.
        std::vector<Feature2D2D> correspondences;
        for (int j = 0; j < num_inliers; j++) {
            // Make sure the point is in front of the camera.
            const Vector3d point_3d = Vector3d::Random() + Vector3d(0, 0, 100);
            const Vector3d proj_3d = gt_rotation * point_3d + gt_translation;

            Feature2D2D corr;
            corr.feature1.pos = point_3d.hnormalized();
            corr.feature2.pos = proj_3d.hnormalized();
            correspondences.emplace_back(corr);
        }

        // Add outliers
        for (int j = 0; j < num_outliers; j++) {
            // Make sure the point is in front of the camera.
            const Vector3d point_3d = Vector3d::Random() + Vector3d(0, 0, -100);
            const Vector3d proj_3d = gt_rotation * point_3d + gt_translation;

            Feature2D2D corr;
            corr.feature1.pos = point_3d.hnormalized();
            corr.feature2.pos = proj_3d.hnormalized();
            correspondences.emplace_back(corr);
        }

        Matrix3d estimated_rotation;
        Vector3d estimated_position;
        const int num_points_in_front = GetBestPoseFromEssentialMatrix(
            ematrix, correspondences, &estimated_rotation, &estimated_position);

        // Ensure that the results are correct. Sincer there is no noise we can
        // expect te number of point in front to be exact.
        EXPECT_EQ(num_points_in_front, num_inliers);
        EXPECT_LT((gt_rotation - estimated_rotation).norm(), kTolerance);
        EXPECT_LT((gt_position - estimated_position).norm(), kTolerance);
    }
}

TEST(GetBestPoseFromEssentialMatrix, AllInliers)
{
    constexpr int kNumInliers = 100;
    constexpr int kNumOutliers = 0;
    TestGetBestPoseFromEssentialMatrix(kNumInliers, kNumOutliers);
}

TEST(GetBestPoseFromEssentialMatrix, MostlyInliers)
{
    constexpr int kNumInliers = 100;
    constexpr int kNumOutliers = 50;
    TestGetBestPoseFromEssentialMatrix(kNumInliers, kNumOutliers);
}

TEST(EpipolarBasics, EstimateFocalLength)
{
    constexpr double kTolerance = 1e-6;
    constexpr auto kSampleCount{100};

    for (auto i{0}; i < kSampleCount; i++) {
        const Vector3d rvec = Vector3d::Random();
        const Matrix3d R =
            AngleAxisd{rvec.norm(), rvec.normalized()}.toRotationMatrix();
        const Vector3d t = Vector3d::Random().normalized();

        // Create calibration matrices.
        constexpr auto kFocalLength1 = 800.0;
        constexpr auto kFocalLength2 = 1000.0;
        Matrix3d fmatrix;
        ComposeFundamentalMatrix(kFocalLength1, kFocalLength2, R.data(),
                                 t.data(), fmatrix.data());

        double focalLength1_est, focalLength2_est;
        EXPECT_TRUE(FocalLengthsFromFundamentalMatrix(
            fmatrix.data(), &focalLength1_est, &focalLength2_est));
        EXPECT_NEAR(focalLength1_est, kFocalLength1, kTolerance);
        EXPECT_NEAR(focalLength2_est, kFocalLength2, kTolerance);
    }
}

TEST(EpipolarBasics, EstimateFocalLengthSharedFocalLengthsZeroIntrinsics)
{
    constexpr double kTolerance = 1e-4;
    constexpr auto kSampleCount{100};

    for (auto i{0}; i < kSampleCount; i++) {
        const Vector3d rvec = Vector3d::Random();
        const Matrix3d R =
            AngleAxisd{rvec.norm(), rvec.normalized()}.toRotationMatrix();
        const Vector3d t = Vector3d::Random().normalized();

        // Create calibration matrices.
        const auto focal = kRNG.randFloat(800., 1600.);
        Matrix3d F;
        ComposeFundamentalMatrix(focal, focal, R.data(), t.data(), F.data());

        double focal_est;
        EXPECT_TRUE(
            SharedFocalLengthsFromFundamentalMatrix(F.data(), &focal_est));
        EXPECT_NEAR(focal_est, focal, kTolerance);
    }
}

TEST(EpipolarBasics, FundamentalMatrixFromProjectionMatrices)
{
    constexpr double kTolerance = 1e-12;
    constexpr int kNumPoints = 10;
    constexpr auto kSampleCount{100};

    for (auto i{0}; i < kSampleCount; ++i) {
        Vector3d objPoints[kNumPoints];
        for (auto j{0}; j < kNumPoints; ++j) {
            objPoints[j] = Vector3d::Random() + Vector3d{0., 0., 10.};
        }

        // Set up projection matrices.
        const Vector3d rvec = Vector3d::Random();
        const Matrix3d R =
            AngleAxisd{rvec.norm(), rvec.normalized()}.toRotationMatrix();
        const Vector3d t = Vector3d::Random();

        Matrix34d pmatrix1, pmatrix2;
        pmatrix1 << Matrix3d::Identity(), Vector3d::Zero();
        pmatrix2 << R, t;

        // Get the fundamental matrix.
        Matrix3d fmatrix;
        FundamentalMatrixFromProjectionMatrices(
            pmatrix1.data(), pmatrix2.data(), fmatrix.data());
        // Ensure the epipolar constraint holds.
        for (auto j{0}; j < kNumPoints; j++) {
            const auto imgPoint1 = pmatrix1 * objPoints[j].homogeneous();
            const auto imgPoint2 = pmatrix2 * objPoints[j].homogeneous();

            EXPECT_LT(std::abs(imgPoint1.transpose() * fmatrix * imgPoint2),
                      kTolerance);
        }
    }
}
