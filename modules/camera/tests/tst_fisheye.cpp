#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <tCamera/FisheyeCameraModel>

using namespace tl;

using Eigen::Vector2d;
using Eigen::Vector3d;

TEST(FisheyeCameraModel, InternalParameterGettersAndSetters)
{
    FisheyeCameraModel camera;

    EXPECT_EQ(camera.type(), CameraIntrinsics::Type::Fisheye);

    // Check that default values are set
    EXPECT_EQ(camera.focalLength(), 1.0);
    EXPECT_EQ(camera.aspectRatio(), 1.0);
    EXPECT_EQ(camera.skew(), 0.0);
    EXPECT_EQ(camera.principalPointX(), 0.0);
    EXPECT_EQ(camera.principalPointY(), 0.0);
    EXPECT_EQ(camera.radialDistortion1(), 0.0);
    EXPECT_EQ(camera.radialDistortion2(), 0.0);
    EXPECT_EQ(camera.radialDistortion3(), 0.0);
    EXPECT_EQ(camera.radialDistortion4(), 0.0);

    // Set parameters to different values.
    camera.setFocalLength(600.0);
    camera.setAspectRatio(0.9);
    camera.setSkew(0.01);
    camera.setPrincipalPoint(300.0, 400.0);
    camera.setRadialDistortion(0.01, 0.001, 0.002, 0.003);

    // Check that the values were updated.
    EXPECT_EQ(camera.focalLength(), 600.0);
    EXPECT_EQ(camera.aspectRatio(), 0.9);
    EXPECT_EQ(camera.skew(), 0.01);
    EXPECT_EQ(camera.principalPointX(), 300.0);
    EXPECT_EQ(camera.principalPointY(), 400.0);
    EXPECT_EQ(camera.radialDistortion1(), 0.01);
    EXPECT_EQ(camera.radialDistortion2(), 0.001);
    EXPECT_EQ(camera.radialDistortion3(), 0.002);
    EXPECT_EQ(camera.radialDistortion4(), 0.003);
}

// Test to ensure that the camera intrinsics are being set appropriately.
inline void TestSetFromCameraintrinsicsPrior(const CameraMetaData& prior)
{
    const FisheyeCameraModel default_camera;
    FisheyeCameraModel camera;
    camera.setFromMetaData(prior);

    if (prior.focal_length.is_set) {
        EXPECT_EQ(camera.focalLength(), prior.focal_length.value[0]);
    }
    else {
        EXPECT_EQ(camera.focalLength(), default_camera.focalLength());
    }

    if (prior.principal_point.is_set) {
        EXPECT_EQ(camera.principalPointX(), prior.principal_point.value[0]);
        EXPECT_EQ(camera.principalPointY(), prior.principal_point.value[1]);
    }
    else {
        EXPECT_EQ(camera.principalPointX(), default_camera.principalPointX());
        EXPECT_EQ(camera.principalPointY(), default_camera.principalPointY());
    }

    if (prior.aspect_ratio.is_set) {
        EXPECT_EQ(camera.aspectRatio(), prior.aspect_ratio.value[0]);
    }
    else {
        EXPECT_EQ(camera.aspectRatio(), default_camera.aspectRatio());
    }

    if (prior.skew.is_set) {
        EXPECT_EQ(camera.skew(), prior.skew.value[0]);
    }
    else {
        EXPECT_EQ(camera.skew(), default_camera.skew());
    }

    if (prior.radial_distortion.is_set) {
        EXPECT_EQ(camera.radialDistortion1(), prior.radial_distortion.value[0]);
        EXPECT_EQ(camera.radialDistortion2(), prior.radial_distortion.value[1]);
        EXPECT_EQ(camera.radialDistortion3(), prior.radial_distortion.value[2]);
        EXPECT_EQ(camera.radialDistortion4(), prior.radial_distortion.value[3]);
    }
    else {
        EXPECT_EQ(camera.radialDistortion1(),
                  default_camera.radialDistortion1());
        EXPECT_EQ(camera.radialDistortion2(),
                  default_camera.radialDistortion2());
        EXPECT_EQ(camera.radialDistortion3(),
                  default_camera.radialDistortion3());
        EXPECT_EQ(camera.radialDistortion4(),
                  default_camera.radialDistortion4());
    }
}

// Gradually add one prior at a time and ensure that the method still works. We
// test before and after setting the "is_set" member variable to true to ensure
// that setting the value of priors when is_set=false is handled properly.
TEST(FisheyeCameraModel, SetFromCameraIntrinsicsPriors)
{
    CameraMetaData prior;
    prior.focal_length.value[0] = 1000.0;
    prior.principal_point.value[0] = 400.0;
    prior.principal_point.value[1] = 300.0;
    prior.aspect_ratio.value[0] = 1.01;
    prior.skew.value[0] = 0.01;
    prior.radial_distortion.value[0] = 0.01;
    prior.radial_distortion.value[1] = 0.001;
    prior.radial_distortion.value[2] = 0.001;
    prior.radial_distortion.value[3] = 0.001;

    TestSetFromCameraintrinsicsPrior(prior);

    prior.focal_length.is_set = true;
    TestSetFromCameraintrinsicsPrior(prior);

    prior.principal_point.is_set = true;
    TestSetFromCameraintrinsicsPrior(prior);

    prior.aspect_ratio.is_set = true;
    TestSetFromCameraintrinsicsPrior(prior);

    prior.skew.is_set = true;
    TestSetFromCameraintrinsicsPrior(prior);

    prior.radial_distortion.is_set = true;
    TestSetFromCameraintrinsicsPrior(prior);
}

TEST(FisheyeCameraModel, constantParameterIndices)
{
    FisheyeCameraModel camera;
    std::vector<int> constant_subset;

    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::None);
    EXPECT_EQ(constant_subset.size(), camera.numParameters());

    // Test that optimizing for focal length works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::FocalLength);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], FisheyeCameraModel::Fx);
    }

    // Test that optimizing for principal_points works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::PrincipalPoint);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 2);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], FisheyeCameraModel::Cx);
        EXPECT_NE(constant_subset[i], FisheyeCameraModel::Cy);
    }

    // Test that optimizing for aspect ratio works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::AspectRatio);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], FisheyeCameraModel::YX);
    }

    // Test that optimizing for skew works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::Skew);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], FisheyeCameraModel::Skew);
    }

    // Test that optimizing for radial distortion works correctly.
    constant_subset = camera.constantParameterIndices(
        OptimizeIntrinsicsType::RadialDistortion);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 4);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], FisheyeCameraModel::K1);
        EXPECT_NE(constant_subset[i], FisheyeCameraModel::K2);
        EXPECT_NE(constant_subset[i], FisheyeCameraModel::K3);
        EXPECT_NE(constant_subset[i], FisheyeCameraModel::K4);
    }

    // Test that optimizing for tangential distortion does not optimize any
    // parameters.
    constant_subset = camera.constantParameterIndices(
        OptimizeIntrinsicsType::TangentialDistortion);
    EXPECT_EQ(constant_subset.size(), camera.numParameters());
}

inline void ReprojectionTest(const FisheyeCameraModel& camera)
{
    constexpr double kTolerance = 1e-5;
    const double kNormalizedTolerance = kTolerance / camera.focalLength();
    constexpr int kImageWidth = 1200;
    constexpr int kImageHeight = 980;
    constexpr double kMinDepth = 2.0;
    constexpr double kMaxDepth = 25.0;

    // Ensure the image -> camera -> image transformation works.
    for (double x = 0.0; x < kImageWidth; x += 10.0) {
        for (double y = 0.0; y < kImageHeight; y += 10.0) {
            const Eigen::Vector2d pixel(x, y);
            // Get the normalized ray of that pixel.
            const Vector3d normalized_ray = camera.imageToSpace(pixel);

            // Test the reprojection at several depths.
            for (double depth = kMinDepth; depth < kMaxDepth; depth += 1.0) {
                // Convert it to a full 3D point in the camera coordinate
                // system.
                const Vector3d point = normalized_ray * depth;
                const Vector2d reprojected_pixel = camera.spaceToImage(point);

                // Expect the reprojection to be close.
                EXPECT_LT((pixel - reprojected_pixel).norm(), kTolerance)
                    << "gt pixel: " << pixel.transpose()
                    << "\n"
                       "reprojected pixel: "
                    << reprojected_pixel.transpose();
            }
        }
    }

    // Ensure the camera -> image -> camera transformation works.
    for (double x = -0.8; x < 0.8; x += 0.1) {
        for (double y = -0.8; y < 0.8; y += 0.1) {
            for (double depth = kMinDepth; depth < kMaxDepth; depth += 1.0) {
                const Eigen::Vector3d point(x, y, depth);
                const Vector2d pixel = camera.spaceToImage(point);

                // Get the normalized ray of that pixel.
                const Vector3d normalized_ray = camera.imageToSpace(pixel);

                // Convert it to a full 3D point in the camera coordinate
                // system.
                const Vector3d reprojected_point = normalized_ray * depth;

                // Expect the reprojection to be close.
                EXPECT_LT((point - reprojected_point).norm(),
                          kNormalizedTolerance)
                    << "gt pixel: " << point.transpose()
                    << "\nreprojected pixel: " << reprojected_point.transpose();
            }
        }
    }
}

TEST(FisheyeCameraModel, ReprojectionNoDistortion)
{
    constexpr double kPrincipalPoint[2] = {600.0, 400.0};
    constexpr double kFocalLength = 1200;
    FisheyeCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(0, 0, 0, 0);
    ReprojectionTest(camera);
}

TEST(FisheyeCameraModel, ReprojectionOneDistortion)
{
    constexpr double kPrincipalPoint[2] = {600.0, 400.0};
    constexpr double kFocalLength = 1200;
    FisheyeCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(0.01, 0.0, 0.0, 0.0);
    ReprojectionTest(camera);
}

TEST(FisheyeCameraModel, ReprojectionTwoDistortion)
{
    constexpr double kPrincipalPoint[2] = {600.0, 400.0};
    constexpr double kFocalLength = 1200;
    FisheyeCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(0.01, 0.001, 0.0, 0.0);
    ReprojectionTest(camera);
}

TEST(FisheyeCameraModel, ReprojectionThreeDistortion)
{
    constexpr double kPrincipalPoint[2] = {600.0, 400.0};
    constexpr double kFocalLength = 1200;
    FisheyeCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(0.01, 0.001, 0.001, 0.0);
    ReprojectionTest(camera);
}

TEST(FisheyeCameraModel, ReprojectionFourDistortion)
{
    constexpr double kPrincipalPoint[2] = {600.0, 400.0};
    constexpr double kFocalLength = 1200;
    FisheyeCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(0.01, 0.001, 0.001, 0.001);
    ReprojectionTest(camera);
}
