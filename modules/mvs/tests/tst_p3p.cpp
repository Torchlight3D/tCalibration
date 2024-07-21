#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <tCore/Math>
#include <tMath/Eigen/Types>
#include <tMvs/PnP/P3P>

#include "test_utils.h"

using namespace tl;

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {
constexpr auto kFocalLength{800.};
}

void P3PTest(const std::optional<double>& noise = {})
{
    // Prepare data
    const Matrix3d rotation =
        (AngleAxisd{math::degToRad(15.), Vector3d::UnitX()} *
         AngleAxisd{math::degToRad(-10.), Vector3d::UnitY()})
            .toRotationMatrix();
    const Vector3d translation{0.3, -1.7, 1.15};
    Matrix34d P;
    P << rotation, translation;

    const std::vector objectPoints{Vector3d{-0.3001, -0.5840, 1.2271},
                                   Vector3d{-1.4487, 0.6965, 0.3889},
                                   Vector3d{-0.7815, 0.7642, 0.1257}};

    std::vector<Vector2d> imagePoints(3);
    for (int i = 0; i < 3; i++) {
        imagePoints[i] = (P * objectPoints[i].homogeneous()).hnormalized();
        if (noise.has_value()) {
            AddNoiseToVector2(noise.value(), &imagePoints[i]);
        }
    }

    // Run P3P
    std::vector<Matrix3d> rotations_est;
    std::vector<Vector3d> translations_est;
    EXPECT_TRUE(
        P3P(imagePoints, objectPoints, rotations_est, translations_est));

    constexpr auto kMaxRpe{2.};       // px
    constexpr auto kMaxAngleDiff{1.}; // deg
    constexpr auto kMaxTranslationDiff{0.1};

    bool valid{false};
    for (size_t i{0}; i < rotations_est.size(); ++i) {
        Matrix34d P_est;
        P_est << rotations_est[i], translations_est[i];

        // Check Rpe
        for (size_t j{0}; j < objectPoints.size(); ++j) {
            const Vector2d reproj =
                (P_est * objectPoints[j].homogeneous()).hnormalized();
            EXPECT_LT((imagePoints[j] - reproj).norm() * kFocalLength, kMaxRpe);
        }

        // Check if the solution is accurate.
        const double rotationDiff =
            math::radToDeg(Quaterniond{rotations_est[i]}.angularDistance(
                Quaterniond{rotation}));
        const double translationDiff =
            ((-rotation * translation) -
             (-rotations_est[i] * translations_est[i]))
                .norm();

        if ((rotationDiff < kMaxAngleDiff) &&
            (translationDiff < kMaxTranslationDiff)) {
            valid = true;
            break;
        }
    }

    EXPECT_TRUE(valid);
}

TEST(P3P, NoNoise) { P3PTest(); }

TEST(P3P, WithNoise) { P3PTest(1. / kFocalLength); }
