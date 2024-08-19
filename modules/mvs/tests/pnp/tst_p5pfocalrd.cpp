#include <random>

#include <Eigen/Geometry>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <tMath/Eigen/Types>
#include <tMvs/PnP/P5PFocalRd>

using namespace tl;

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

using Array5d = Eigen::Array<double, 1, 5>;
using Matrix25d = Eigen::Matrix<double, 2, 5>;
using Matrix35d = Eigen::Matrix<double, 3, 5>;

inline void P5pfrTestWithNoise(
    const Eigen::Matrix3d& gt_rotation, const Eigen::Vector3d& gt_translation,
    double focal_length, double radial_distortion,
    const std::vector<Eigen::Vector3d>& world_points_vector, double noise,
    double reproj_tolerance)
{
    Eigen::Map<const Matrix35d> world_points(world_points_vector[0].data());

    // Camera intrinsics matrix.
    const Matrix3d camera_matrix =
        Eigen::DiagonalMatrix<double, 3>(1.0, 1.0, 1.0 / focal_length);
    // Create the projection matrix P = K * [R t].
    Matrix34d gt_projection;
    gt_projection << gt_rotation, gt_translation;
    gt_projection = camera_matrix * gt_projection;

    // Reproject 3D points to get undistorted image points.
    Matrix25d undistorted_image_point =
        (gt_projection * world_points.colwise().homogeneous())
            .colwise()
            .hnormalized();

    // Determine radius of undistorted points and use that to compute the radius
    // of the distorted points.
    Array5d radius_undistorted = undistorted_image_point.colwise().norm();
    Array5d radius_distorted =
        (1. -
         (1. - 4. * radial_distortion * radius_undistorted.square()).sqrt()) /
        (2. * radial_distortion * radius_undistorted);
    Array5d distortion_vec = radius_distorted / radius_undistorted;

    // Apply radial distortion.
    std::vector<Vector2d> distorted_image_points_vector(5);
    Eigen::Map<Matrix25d> distorted_image_point(
        distorted_image_points_vector[0].data());
    distorted_image_point = undistorted_image_point.cwiseProduct(
        distortion_vec.matrix().replicate<2, 1>());

    // Add noise to distorted image points.
    if (noise) {
        std::default_random_engine generator;
        std::normal_distribution<double> distribution(0.0, noise);
        for (int i = 0; i < 5; i++) {
            distorted_image_point.col(i).x() += distribution(generator);
            distorted_image_point.col(i).y() += distribution(generator);
        }
    }

    // Run P5Pfr algorithm.
    std::vector<Matrix34d> soln_projection;
    std::vector<std::vector<double>> soln_distortion;
    CHECK(FivePointFocalLengthRadialDistortion(
        distorted_image_points_vector, world_points_vector, 1, &soln_projection,
        &soln_distortion));

    bool matched_transform = false;
    for (int i = 0; i < 4; ++i) {
        matched_transform = true;
        // Check the reprojection error.
        for (int n = 0; n < 5; n++) {
            const double distortion_w =
                1.0 + soln_distortion[i][0] *
                          distorted_image_point.col(n).squaredNorm();
            Vector2d undist_pt = distorted_image_point.col(n) / distortion_w;
            Vector3d reproj_pt =
                soln_projection[i] * world_points.col(n).homogeneous();
            const double reproj_error =
                (undist_pt - reproj_pt.hnormalized()).squaredNorm();
            if (reproj_error > reproj_tolerance) {
                matched_transform = false;
                break;
            }
        }

        if (matched_transform) {
            break;
        }
    }
    // One of the solutions must have been a valid solution.
    EXPECT_TRUE(matched_transform);
}

inline void BasicTest(const double noise, const double reproj_tolerance)
{
    // focal length (values used in the ICCV paper)
    const double focal_length = 1.3;
    // radial distortion (values used in the ICCV paper)
    const double radial_distortion = -0.35;

    const double x = -0.10; // rotation of the view around x axis
    const double y = -0.20; // rotation of the view around y axis
    const double z = 0.30;  // rotation of the view around z axis

    // Create a ground truth pose.
    Matrix3d Rz, Ry, Rx;
    Rz << cos(z), sin(z), 0, -sin(z), cos(z), 0, 0, 0, 1;
    Ry << cos(y), 0, -sin(y), 0, 1, 0, sin(y), 0, cos(y);
    Rx << 1, 0, 0, 0, cos(x), sin(x), 0, -sin(x), cos(x);
    const Matrix3d gt_rotation = Rz * Ry * Rx;
    const Vector3d gt_translation =
        Vector3d(-0.00950692, 000.0171496, 000.0508743);

    // Create 3D world points that are viable based on the camera intrinsics and
    // extrinsics.
    std::vector<Vector3d> world_points_vector(5);
    Eigen::Map<Matrix35d> world_points(world_points_vector[0].data());
    // clang-format off
    world_points << -0.42941, 0.000621211, -0.350949, -1.45205,   -1.294,
                    0.415794,   -0.556605,  -1.92898, -1.89976, -1.12445,
                      1.4949,    0.838307,   1.41972,  1.25756, 0.805163;
    // clang-format on
    P5pfrTestWithNoise(gt_rotation, gt_translation, focal_length,
                       radial_distortion, world_points_vector, noise,
                       reproj_tolerance);
}

inline void PlanarTestWithNoise(double noise, double reproj_tolerance)
{
    // focal length (values used in the ICCV paper)
    const double focal_length = 1.3;
    // radial distortion (values used in the ICCV paper)
    const double radial_distortion = -0.35;
    const double size = 100;
    const double depth = 150;

    const double x = -0.10; // rotation of the view around x axis
    const double y = -0.20; // rotation of the view around y axis
    const double z = 0.30;  // rotation of the view around z axis

    // Create a ground truth pose.
    Matrix3d Rz, Ry, Rx;
    Rz << cos(z), sin(z), 0, -sin(z), cos(z), 0, 0, 0, 1;
    Ry << cos(y), 0, -sin(y), 0, 1, 0, sin(y), 0, cos(y);
    Rx << 1, 0, 0, 0, cos(x), sin(x), 0, -sin(x), cos(x);
    const Matrix3d gt_rotation = Rz * Ry * Rx;
    const Vector3d gt_translation =
        Vector3d(-0.00950692, 000.0171496, 000.0508743);

    // Create 3D world points that are viable based on the camera intrinsics and
    // extrinsics.
    std::vector<Vector3d> world_points_vector(5);
    world_points_vector[0] = Vector3d(-size / 2, -size / 2, depth);
    world_points_vector[1] = Vector3d(size / 2, -size / 2, depth);
    world_points_vector[2] = Vector3d(size / 2, size / 2, depth);
    world_points_vector[3] = Vector3d(-size / 2, size / 2, depth);
    world_points_vector[4] = Vector3d(0.0, 0.0, depth);

    P5pfrTestWithNoise(gt_rotation, gt_translation, focal_length,
                       radial_distortion, world_points_vector, noise,
                       reproj_tolerance);
}

TEST(P5Pfr, BasicTest) { BasicTest(0.0, 1e-12); }

TEST(P5Pfr, BasicNoiseTest) { BasicTest(0.5 / 800.0, 5 / 800.0); }

TEST(P5Pfr, PlanarTestNoNoise) { PlanarTestWithNoise(0.0, 1e-12); }

TEST(P5Pfr, PlanarTestWithNoise)
{
    PlanarTestWithNoise(0.5 / 800.0, 5 / 800.0);
}
