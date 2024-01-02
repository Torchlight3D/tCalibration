#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <AxCamera/OrthographicCameraModel>

using namespace thoht;

using Eigen::Vector2d;
using Eigen::Vector3d;

TEST(OrthographicCameraModel, InternalParameterGettersAndSetters)
{
    OrthographicCameraModel camera;

    EXPECT_EQ(camera.type(), CameraIntrinsics::Type::Orthographic);

    // Check that default values are set
    EXPECT_EQ(camera.focalLength(), 1.);
    EXPECT_EQ(camera.aspectRatio(), 1.);
    EXPECT_EQ(camera.skew(), 0.);
    EXPECT_EQ(camera.principalPointX(), 0.);
    EXPECT_EQ(camera.principalPointY(), 0.);
    EXPECT_EQ(camera.radialDistortion1(), 0.);
    EXPECT_EQ(camera.radialDistortion2(), 0.);

    // Set parameters to different values.
    camera.setFocalLength(30000);
    camera.setAspectRatio(1.1);
    camera.setSkew(0.1);
    camera.setPrincipalPoint(300., 400.);
    camera.setRadialDistortion(0.01, 0.001);

    // Check that the values were updated.
    EXPECT_EQ(camera.focalLength(), 30000);
    EXPECT_EQ(camera.aspectRatio(), 1.1);
    EXPECT_EQ(camera.skew(), 0.1);
    EXPECT_EQ(camera.principalPointX(), 300.);
    EXPECT_EQ(camera.principalPointY(), 400.);
    EXPECT_EQ(camera.radialDistortion1(), 0.01);
    EXPECT_EQ(camera.radialDistortion2(), 0.001);
}

// Test to ensure that the camera intrinsics are being set appropriately.
inline void TestSetFromCameraintrinsicsPrior(const CameraMetaData& meta)
{
    const OrthographicCameraModel default_camera;
    OrthographicCameraModel camera;
    camera.setFromMetaData(meta);

    if (meta.focal_length.is_set) {
        EXPECT_EQ(camera.focalLength(), meta.focal_length.value[0]);
    }
    else {
        EXPECT_EQ(camera.focalLength(), default_camera.focalLength());
    }

    if (meta.principal_point.is_set) {
        EXPECT_EQ(camera.principalPointX(), meta.principal_point.value[0]);
        EXPECT_EQ(camera.principalPointY(), meta.principal_point.value[1]);
    }
    else {
        EXPECT_EQ(camera.principalPointX(), default_camera.principalPointX());
        EXPECT_EQ(camera.principalPointY(), default_camera.principalPointY());
    }

    if (meta.aspect_ratio.is_set) {
        EXPECT_EQ(camera.aspectRatio(), meta.aspect_ratio.value[0]);
    }
    else {
        EXPECT_EQ(camera.aspectRatio(), default_camera.aspectRatio());
    }

    if (meta.skew.is_set) {
        EXPECT_EQ(camera.skew(), meta.skew.value[0]);
    }
    else {
        EXPECT_EQ(camera.skew(), default_camera.skew());
    }

    if (meta.radial_distortion.is_set) {
        EXPECT_EQ(camera.radialDistortion1(), meta.radial_distortion.value[0]);
        EXPECT_EQ(camera.radialDistortion2(), meta.radial_distortion.value[1]);
    }
    else {
        EXPECT_EQ(camera.radialDistortion1(),
                  default_camera.radialDistortion1());
        EXPECT_EQ(camera.radialDistortion2(),
                  default_camera.radialDistortion2());
    }
}

// Gradually add one prior at a time and ensure that the method still works. We
// test before and after setting the "is_set" member variable to true to ensure
// that setting the value of priors when is_set=false is handled properly.
TEST(OrthographicCameraModel, SetFromCameraIntrinsicsPriors)
{
    CameraMetaData meta;
    meta.focal_length.value[0] = 30000;
    meta.principal_point.value[0] = 400.0;
    meta.principal_point.value[1] = 300.0;
    meta.skew.value[0] = 0.0;
    meta.aspect_ratio.value[0] = 1.001;
    meta.radial_distortion.value[0] = 0.01;
    meta.radial_distortion.value[1] = 0.001;

    TestSetFromCameraintrinsicsPrior(meta);

    meta.focal_length.is_set = true;
    TestSetFromCameraintrinsicsPrior(meta);

    meta.principal_point.is_set = true;
    TestSetFromCameraintrinsicsPrior(meta);

    meta.aspect_ratio.is_set = true;
    TestSetFromCameraintrinsicsPrior(meta);

    meta.skew.is_set = true;
    TestSetFromCameraintrinsicsPrior(meta);

    meta.radial_distortion.is_set = true;
    TestSetFromCameraintrinsicsPrior(meta);
}

TEST(OrthographicCameraModel, GetSubsetFromOptimizeIntrinsicsType)
{
    OrthographicCameraModel camera;
    std::vector<int> constant_subset;

    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::None);
    EXPECT_EQ(constant_subset.size(), camera.numParameters());

    // Test that optimizing for magnification works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::FocalLength);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (size_t i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], OrthographicCameraModel::Fx);
    }

    // Test that optimizing for principal_points works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::PrincipalPoint);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 2);
    for (size_t i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], OrthographicCameraModel::Cx);
        EXPECT_NE(constant_subset[i], OrthographicCameraModel::Cy);
    }

    // Test that optimizing for pixel pitches works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::AspectRatio);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (size_t i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], OrthographicCameraModel::YX);
    }

    // Test that optimizing for pixel pitches works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::Skew);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (size_t i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], OrthographicCameraModel::Skew);
    }

    // Test that optimizing for radial distortion works correctly.
    constant_subset = camera.constantParameterIndices(
        OptimizeIntrinsicsType::RadialDistortion);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 2);
    for (size_t i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], OrthographicCameraModel::K1);
        EXPECT_NE(constant_subset[i], OrthographicCameraModel::K2);
    }

    // Test that optimizing for tangential distortion does not optimize any
    // parameters.
    constant_subset = camera.constantParameterIndices(
        OptimizeIntrinsicsType::TangentialDistortion);
    EXPECT_EQ(constant_subset.size(), camera.numParameters());
}

// Test the projection functions of the camera model by testing pixels on a grid
// in the entire image to ensure that the effects of lens distortion at the
// edges of the image are modeled correctly.
void ReprojectionTest(const OrthographicCameraModel& camera)
{
    constexpr double kTolerance = 1e-5;
    constexpr double kNormalizedTolerance = kTolerance;
    constexpr int kImageWidth = 2560;
    constexpr int kImageHeight = 1920;

    // Ensure the camera -> image -> camera transformation works.
    for (double x = -0.01; x < 0.01; x += 0.001) {
        for (double y = -0.01; y < 0.01; y += 0.001) {
            const Eigen::Vector3d point(x, y, 1.0);
            const Vector2d pixel = camera.spaceToImage(point);

            // Get the normalized ray of that pixel.
            const Vector3d p_c = camera.imageToSpace(pixel);

            // Expect the reprojection to be close.
            EXPECT_LT((point - p_c).norm(), kNormalizedTolerance)
                << "gt pixel: " << point.transpose()
                << "\nreprojected pixel: " << p_c.transpose();
        }
    }

    // Ensure the image -> camera -> image transformation works.
    for (double x = 0.0; x < kImageWidth; x += 20.0) {
        for (double y = 0.0; y < kImageHeight; y += 20.0) {
            const Eigen::Vector2d pixel(x, y);
            // Get the normalized ray of that pixel.
            const Vector3d p_c = camera.imageToSpace(pixel);

            // Test the reprojection at several depths.
            // Convert it to a full 3D point in the camera coordinate system.
            const Vector2d reprojected_pixel = camera.spaceToImage(p_c);

            // Expect the reprojection to be close.
            EXPECT_LT((pixel - reprojected_pixel).norm(), kTolerance)
                << "gt pixel: " << pixel.transpose()
                << "\nreprojected pixel: " << reprojected_pixel.transpose();
        }
    }
}

TEST(OrthographicCameraModel, ReprojectionNoDistortion)
{
    constexpr double kPrincipalPoint[2] = {1180.0, 1010.0};
    // this is actually magnification / pixel_pitch, e.g. 0.08/2e-6
    constexpr double kFocalLength = 40000.0;
    OrthographicCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(0, 0);
    ReprojectionTest(camera);
}

TEST(OrthographicCameraModel, ReprojectionOneDistortion)
{
    constexpr double kPrincipalPoint[2] = {1180.0, 1010.0};
    constexpr double kFocalLength = 40000.0;
    OrthographicCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(1e-4, 0);
    ReprojectionTest(camera);
}

TEST(OrthographicCameraModel, ReprojectionTwoDistortion)
{
    constexpr double kPrincipalPoint[2] = {1180.0, 1010.0};
    constexpr double kFocalLength = 40000.0;
    OrthographicCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(1e-4, 1e-5);
    ReprojectionTest(camera);
}
