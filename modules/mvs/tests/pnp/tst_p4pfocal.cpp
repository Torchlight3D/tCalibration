#include <Eigen/Core>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <tCore/RandomGenerator>
#include <tMath/Eigen/Utils>

#include <tMvs/PnP/P4PFocal>

using namespace tl;

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector4d;

using Matrix24d = Eigen::Matrix<double, 2, 4>;

namespace {
RandomNumberGenerator rng(151);
}

inline void P4pfTestWithNoise(
    const Eigen::Matrix3d& gt_rotation, const Eigen::Vector3d& gt_translation,
    double focal_length,
    const std::vector<Eigen::Vector3d>& world_points_vector, double noise,
    double reproj_tolerance)
{
    Eigen::Map<const Matrix34d> world_points(world_points_vector[0].data());

    // Camera intrinsics matrix.
    const Matrix3d camera_matrix =
        Eigen::DiagonalMatrix<double, 3>(focal_length, focal_length, 1.0);
    // Create the projection matrix P = K * [R t].
    Matrix34d gt_projection;
    gt_projection << gt_rotation, gt_translation;
    gt_projection = camera_matrix * gt_projection;

    // Reproject 3D points to get undistorted image points.
    std::vector<Eigen::Vector2d> image_points_vector(5);
    Eigen::Map<Matrix24d> image_point(image_points_vector[0].data());
    image_point = (gt_projection * world_points.colwise().homogeneous())
                      .colwise()
                      .hnormalized();

    // Add noise to distorted image points.
    if (noise) {
        std::default_random_engine generator;
        std::normal_distribution<double> distribution(0.0, noise);
        for (int i = 0; i < 4; i++) {
            image_point.col(i).x() += distribution(generator);
            image_point.col(i).y() += distribution(generator);
        }
    }

    // Run P5pf algorithm.
    std::vector<Matrix34d> soln_projection;
    int num_solns = FourPointPoseAndFocalLength(
        image_points_vector, world_points_vector, soln_projection);
    ASSERT_GT(num_solns, 0);

    bool matched_transform = false;
    for (int i = 0; i < num_solns; ++i) {
        matched_transform = true;
        // Check that the reprojection error is very small.
        for (int n = 0; n < 4; n++) {
            Vector3d reproj_point =
                soln_projection[i] * world_points.col(n).homogeneous();
            const double reproj_error =
                (reproj_point.hnormalized() - image_point.col(n)).norm();
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
    const double focal_length = 800;

    const double x = -0.10; // rotation of the view around x axis
    const double y = -0.20; // rotation of the view around y axis
    const double z = 0.30;  // rotation of the view around z axis

    // Create a ground truth pose.
    Matrix3d Rz, Ry, Rx;
    Rz << cos(z), sin(z), 0, -sin(z), cos(z), 0, 0, 0, 1;
    Ry << cos(y), 0, -sin(y), 0, 1, 0, sin(y), 0, cos(y);
    Rx << 1, 0, 0, 0, cos(x), sin(x), 0, -sin(x), cos(x);
    const Matrix3d gt_rotation = Rz * Ry * Rx;
    const Vector3d gt_translation = Vector3d(-0.00950692, 0.0171496, 0.0508743);

    // Create 3D world points that are viable based on the camera intrinsics and
    // extrinsics.
    std::vector<Vector3d> world_points_vector = {
        Vector3d(-1.0, 0.5, 1.2), Vector3d(-0.79, -0.68, 1.9),
        Vector3d(1.42, 1.01, 2.19), Vector3d(0.87, -0.49, 0.89)};

    P4pfTestWithNoise(gt_rotation, gt_translation, focal_length,
                      world_points_vector, noise, reproj_tolerance);
}

inline void RandomTestWithNoise(const double noise,
                                const double reproj_tolerance)
{
    const double kBaseline = 0.25;

    // focal length (values used in the ICCV paper)
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    const double focal_length = distribution(generator) * 50.0 + 600;

    // Rotation areound x, y, z axis.
    const double x = distribution(generator) * 0.5 - 0.25;
    const double y = distribution(generator) * 0.5 - 0.25;
    const double z = distribution(generator) * 0.5 - 0.25;

    // Create a ground truth pose.
    Matrix3d Rz, Ry, Rx;
    Rz << cos(z), sin(z), 0, -sin(z), cos(z), 0, 0, 0, 1;
    Ry << cos(y), 0, -sin(y), 0, 1, 0, sin(y), 0, cos(y);
    Rx << 1, 0, 0, 0, cos(x), sin(x), 0, -sin(x), cos(x);
    const Matrix3d gt_rotation = Rz * Ry * Rx;
    const Vector3d gt_translation = Vector3d::Random() * kBaseline;

    // Create 3D world points that are viable based on the camera intrinsics and
    // extrinsics.
    std::vector<Vector3d> world_points_vector(4);
    Eigen::Map<Matrix34d> world_points(world_points_vector[0].data());
    world_points.row(2) = 2.0 * Vector4d::Random().transpose().array() + 2.0;
    world_points.row(1) = 2.0 * Vector4d::Random().transpose();
    world_points.row(0) = 2.0 * Vector4d::Random().transpose();

    P4pfTestWithNoise(gt_rotation, gt_translation, focal_length,
                      world_points_vector, noise, reproj_tolerance);
}

TEST(P4pf, BasicTest) { BasicTest(0.0, 1e-4); }

TEST(P4pf, BasicNoiseTest) { BasicTest(0.5, 10.0); }

TEST(P4pf, RandomTest) { RandomTestWithNoise(0.0, 0.1); }
