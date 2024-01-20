#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <tMath/EigenUtils>
#include <tMath/RandomGenerator>
#include <tCamera/Camera>
#include <tCamera/CameraMetaData>
#include <tCamera/PinholeCameraModel>

using namespace tl;

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

RandomNumberGenerator rng{157};

TEST(Camera, ProjectionMatrix)
{
    constexpr double kTolerance = 1e-12;

    Camera camera;
    constexpr double image_size = 500.;
    Matrix34d gt_projection_matrix;
    for (int i = 0; i < 100; i++) {
        rng.SetRandom(&gt_projection_matrix);
        EXPECT_TRUE(camera.setFromProjectMatrix(image_size, image_size,
                                                gt_projection_matrix));
        Matrix34d projection_matrix;
        camera.projectionMatrix(projection_matrix);

        EXPECT_TRUE(math::ArraysEqualUpToScale(12, gt_projection_matrix.data(),
                                               projection_matrix.data(),
                                               kTolerance));
    }
}

TEST(Camera, InternalParameterGettersAndSetters)
{
    Camera camera;

    auto intrinsics = camera.cameraIntrinsics();

    PinholeCameraModel pinhole_intrinsics;

    // Check that default values are set
    EXPECT_EQ(camera.focalLength(), 1.);
    EXPECT_EQ(camera.principalPointX(), 0.);
    EXPECT_EQ(camera.principalPointY(), 0.);

    // Make sure the default intrinsics are sets for pinhole cameras.
    EXPECT_EQ(camera.cameraIntrinsicsModel(), CameraIntrinsics::Type::Pinhole);
    for (int i = 0; i < intrinsics->numParameters(); i++) {
        EXPECT_EQ(intrinsics->parameter(i), pinhole_intrinsics.parameter(i));
    }

    // Set parameters to different values.
    camera.setFocalLength(600.0);
    camera.setPrincipalPoint(300.0, 400.0);

    // Check that the values were updated.
    EXPECT_EQ(camera.focalLength(), 600.0);
    EXPECT_EQ(camera.principalPointX(), 300.0);
    EXPECT_EQ(camera.principalPointY(), 400.0);
}

TEST(Camera, ExternalParameterGettersAndSetters)
{
    constexpr double kTolerance = 1e-16;

    Camera camera;

    // Check that the default values are set.
    EXPECT_DOUBLE_EQ(camera.position().squaredNorm(), 0.0);
    EXPECT_DOUBLE_EQ(camera.orientationAsAngleAxis().squaredNorm(), 0.0);
    EXPECT_DOUBLE_EQ(
        (camera.orientationAsRotationMatrix() - Matrix3d::Identity())
            .squaredNorm(),
        0.0);

    // Check that position getter/setters work.
    camera.setPosition(Vector3d::Ones());
    EXPECT_DOUBLE_EQ((camera.position() - Vector3d::Ones()).squaredNorm(), 0.0);

    // Check that angle axis getter/setters work.
    Vector3d gt_angle_axis(1.0, 1.0, 1.0);
    gt_angle_axis = Vector3d(0.3, 0.7, 0.4);
    Matrix3d gt_rotation_matrix;
    ceres::AngleAxisToRotationMatrix(gt_angle_axis.data(),
                                     gt_rotation_matrix.data());
    camera.setOrientationFromRotationMatrix(gt_rotation_matrix);
    EXPECT_LT((camera.orientationAsAngleAxis() - gt_angle_axis).squaredNorm(),
              kTolerance);
    EXPECT_LT((camera.orientationAsRotationMatrix() - gt_rotation_matrix)
                  .squaredNorm(),
              kTolerance);

    // Check that rotation matrix getter/setters work.
    gt_angle_axis = Vector3d{0.3, 0.7, 0.4};
    ceres::AngleAxisToRotationMatrix(gt_angle_axis.data(),
                                     gt_rotation_matrix.data());
    camera.setOrientationFromRotationMatrix(gt_rotation_matrix);
    EXPECT_LT((camera.orientationAsAngleAxis() - gt_angle_axis).squaredNorm(),
              kTolerance);
    EXPECT_LT((camera.orientationAsRotationMatrix() - gt_rotation_matrix)
                  .squaredNorm(),
              kTolerance);
}

TEST(Camera, SetFromCameraIntrinsicsPrior)
{
    CameraMetaData meta;
    meta.image_width = 1920;
    meta.image_height = 1080;

    Camera camera;
    camera.setFromMetaData(meta);
    EXPECT_EQ(camera.cameraIntrinsicsModel(), CameraIntrinsics::Type::Pinhole);
    EXPECT_EQ(camera.imageWidth(), meta.image_width);
    EXPECT_EQ(camera.imageHeight(), meta.image_height);

    // TODO: Try wrong string
    // Set the meta for intrinsics model to Pinhole.
    meta.camera_intrinsics_model_type = "Pinhole";
    camera.setFromMetaData(meta);
    EXPECT_EQ(camera.cameraIntrinsicsModel(), CameraIntrinsics::Type::Pinhole);

    // Set the meta for intrinsics model to PinholeRadialTangential.
    meta.camera_intrinsics_model_type = "PinholeRadialTangential";
    camera.setFromMetaData(meta);
    EXPECT_EQ(camera.cameraIntrinsicsModel(),
              CameraIntrinsics::Type::PinholeRadialTangential);

    // Set the meta for intrinsics model to Fisheye.
    meta.camera_intrinsics_model_type = "Fisheye";
    camera.setFromMetaData(meta);
    EXPECT_EQ(camera.cameraIntrinsicsModel(), CameraIntrinsics::Type::Fisheye);
}

void ReprojectionTest(const Camera& camera)
{
    constexpr double kTolerance = 1e-5;

    for (int i = 0; i < 10; i++) {
        // Get a random pixel within the image.
        const Vector2d pixel =
            camera.imageWidth() * rng.RandVector2d(-1.0, 1.0);

        // Get the normalized ray of that pixel.
        const Vector3d normalized_ray = camera.pixelToUnitDepthRay(pixel);

        const double random_depth = rng.RandDouble(0.01, 100.0);
        const Vector4d random_point =
            (camera.position() + normalized_ray * random_depth).homogeneous();

        Vector2d reprojected_pixel;
        const double depth =
            camera.projectPoint(random_point, reprojected_pixel);

        // Expect the reconstructed 3d points to be close.
        EXPECT_LT(std::abs(random_depth - depth), kTolerance * random_depth)
            << "real depth = " << random_depth
            << " and reconstructed depth = " << depth;

        // Expect the reprojection to be close.
        EXPECT_LT((pixel - reprojected_pixel).norm(), kTolerance)
            << "gt pixel: " << pixel.transpose()
            << "\nreprojected pixel: " << reprojected_pixel.transpose();
    }
}

TEST(Camera, Reprojection)
{
    constexpr double image_size = 600.;

    Camera camera;
    Matrix34d projection_mat;
    for (int i = 0; i < 100; i++) {
        // Initialize a random camera.
        rng.SetRandom(&projection_mat);
        camera.setFromProjectMatrix(image_size, image_size, projection_mat);

        ReprojectionTest(camera);
    }
}

TEST(Camera, SetCameraIntrinsicsType)
{
    constexpr double kFocalLength = 100.0;

    Camera camera;
    EXPECT_EQ(camera.cameraIntrinsicsModel(), CameraIntrinsics::Type::Pinhole);
    // Set a camera intrinsics parameter.
    camera.setFocalLength(kFocalLength);
    // Set the camera intrinsics type to be the same as it currently is. This
    // should produce a no-op and the focal length value should be preserved.
    camera.setCameraIntrinsicsModel(CameraIntrinsics::Type::Pinhole);
    EXPECT_EQ(camera.focalLength(), kFocalLength);
    EXPECT_EQ(camera.cameraIntrinsicsModel(), CameraIntrinsics::Type::Pinhole);
}
