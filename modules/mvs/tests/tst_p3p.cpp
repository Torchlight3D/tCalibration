#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
// #include <glog/logging.h>
#include <gtest/gtest.h>

#include <tMath/MathBase>
#include <tMath/RandomGenerator>
#include <tMvs/P3P>
#include "test_utils.h"

using namespace tl;
using namespace tl::math;

using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::AngleAxisd;
using Eigen::Quaterniond;

namespace {
RandomNumberGenerator rng(55);
}

void PoseFromThreeCalibratedTest(double noise)
{
    // Projection matrix.
    const Matrix3d gt_rotation =
        (AngleAxisd{degToRad(15.0), Vector3d::UnitX()} *
         AngleAxisd{degToRad(-10.), Vector3d::UnitY()})
            .toRotationMatrix();
    const Vector3d gt_translation{0.3, -1.7, 1.15};
    Matrix34d projection_mat;
    projection_mat << gt_rotation, gt_translation;

    // Points in the 3D scene.
    Vector3dList kPoints3d{Vector3d(-0.3001, -0.5840, 1.2271),
                           Vector3d(-1.4487, 0.6965, 0.3889),
                           Vector3d(-0.7815, 0.7642, 0.1257)};

    // Points in the camera view.
    Vector2dList kPoints2d(3);
    for (int i = 0; i < 3; i++) {
        kPoints2d[i] =
            (projection_mat * kPoints3d[i].homogeneous()).eval().hnormalized();
        if (!isApprox0(noise)) {
            AddNoiseToProjection(noise, &rng, &kPoints2d[i]);
        }
    }

    Matrix3dList rotations;
    Vector3dList translations;
    EXPECT_TRUE(P3P(kPoints2d, kPoints3d, rotations, translations));

    constexpr double kMaxAngleDiff{1.}; // deg
    constexpr double kMaxTranslationDiff{0.1};

    bool matched_transform{false};
    for (int i = 0; i < rotations.size(); ++i) {
        // Check that the rotation and translation are close.
        double angular_diff =
            radToDeg(Quaterniond{rotations[i]}.angularDistance(
                Quaterniond{gt_rotation}));
        double trans_diff = ((-gt_rotation * gt_translation) -
                             (-rotations[i] * translations[i]))
                                .norm();
        bool rot_match = angular_diff < kMaxAngleDiff;
        bool trans_match = trans_diff < kMaxTranslationDiff;
        if (rot_match && trans_match) {
            matched_transform = true;

            Matrix34d soln_proj;
            soln_proj << rotations[i], translations[i];
            // Check the reprojection error.
            for (int j = 0; j < 3; j++) {
                const Vector3d projected_pt =
                    soln_proj * kPoints3d[j].homogeneous();
                EXPECT_LT(
                    (kPoints2d[j] - projected_pt.hnormalized()).norm() * 800.0,
                    2.0);
            }
        }
    }
    EXPECT_TRUE(matched_transform);
}

TEST(P3P, PoseFromThreeCalibrated) { PoseFromThreeCalibratedTest(0.0 / 800.0); }

TEST(P3P, PoseFromThreeCalibratedNoise)
{
    PoseFromThreeCalibratedTest(1.0 / 800.0);
}
