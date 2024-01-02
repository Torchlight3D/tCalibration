#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <AxCamera/DoubleSphereCameraModel>

using namespace thoht;

using Eigen::Vector2d;
using Eigen::Vector3d;

TEST(DoubleSphereCameraModel, InternalParameterGettersAndSetters)
{
    DoubleSphereCameraModel camera;

    EXPECT_EQ(camera.type(), CameraIntrinsics::Type::DoubleSphere);

    // Check that default values are set
    EXPECT_EQ(camera.focalLength(), 1.0);
    EXPECT_EQ(camera.aspectRatio(), 1.0);
    EXPECT_EQ(camera.skew(), 0.0);
    EXPECT_EQ(camera.principalPointX(), 0.0);
    EXPECT_EQ(camera.principalPointY(), 0.0);
    EXPECT_EQ(camera.alpha(), 0.5);
    EXPECT_EQ(camera.xi(), 0.);

    // Set parameters to different values.
    camera.setFocalLength(0.5 * 805);
    camera.setAspectRatio(805 / 800);
    camera.setSkew(0.00);
    camera.setPrincipalPoint(505, 509);
    camera.setDistortion(0.5 * -0.150694, 0.5 * 1.48785);

    // Check that the values were updated.
    EXPECT_EQ(camera.focalLength(), 0.5 * 805);
    EXPECT_EQ(camera.aspectRatio(), 805 / 800);
    EXPECT_EQ(camera.skew(), 0.00);
    EXPECT_EQ(camera.principalPointX(), 505);
    EXPECT_EQ(camera.principalPointY(), 509);
    EXPECT_EQ(camera.alpha(), 0.5 * -0.150694);
    EXPECT_EQ(camera.xi(), 0.5 * 1.48785);
}

// Test to ensure that the camera intrinsics are being set appropriately.
inline void TestSetFromCameraMetaData(const CameraMetaData& prior)
{
    const DoubleSphereCameraModel default_camera;
    DoubleSphereCameraModel camera;
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
        EXPECT_EQ(camera.alpha(), prior.radial_distortion.value[0]);
        EXPECT_EQ(camera.xi(), prior.radial_distortion.value[1]);
    }
    else {
        EXPECT_EQ(camera.xi(), default_camera.xi());
        EXPECT_EQ(camera.alpha(), default_camera.alpha());
    }
}

// Gradually add one prior at a time and ensure that the method still works. We
// test before and after setting the "is_set" member variable to true to ensure
// that setting the value of priors when is_set=false is handled properly.
TEST(DoubleSphereCameraModel, SetFromCameraMetaDatas)
{
    CameraMetaData prior;
    prior.focal_length.value[0] = 1000.0;
    prior.principal_point.value[0] = 400.0;
    prior.principal_point.value[1] = 300.0;
    prior.aspect_ratio.value[0] = 1.01;
    prior.skew.value[0] = 0.01;
    prior.radial_distortion.value[0] = 0.01;
    prior.radial_distortion.value[1] = 0.001;

    TestSetFromCameraMetaData(prior);

    prior.focal_length.is_set = true;
    TestSetFromCameraMetaData(prior);

    prior.principal_point.is_set = true;
    TestSetFromCameraMetaData(prior);

    prior.aspect_ratio.is_set = true;
    TestSetFromCameraMetaData(prior);

    prior.skew.is_set = true;
    TestSetFromCameraMetaData(prior);

    prior.radial_distortion.is_set = true;
    TestSetFromCameraMetaData(prior);
}

TEST(DoubleSphereCameraModel, GetSubsetFromOptimizeIntrinsicsType)
{
    DoubleSphereCameraModel camera;
    std::vector<int> constant_subset;

    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::None);
    EXPECT_EQ(constant_subset.size(), camera.numParameters());

    // Test that optimizing for focal length works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::FocalLength);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (const auto& constant : constant_subset) {
        EXPECT_NE(constant, DoubleSphereCameraModel::Fx);
    }

    // Test that optimizing for principal_points works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::PrincipalPoint);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 2);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], DoubleSphereCameraModel::Cx);
        EXPECT_NE(constant_subset[i], DoubleSphereCameraModel::Cy);
    }

    // Test that optimizing for aspect ratio works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::AspectRatio);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], DoubleSphereCameraModel::YX);
    }

    // Test that optimizing for skew works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::Skew);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], DoubleSphereCameraModel::Skew);
    }

    // Test that optimizing for radial distortion works correctly.
    constant_subset = camera.constantParameterIndices(
        OptimizeIntrinsicsType::RadialDistortion);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 2);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], DoubleSphereCameraModel::Alpha);
        EXPECT_NE(constant_subset[i], DoubleSphereCameraModel::Xi);
    }
}

// Test the projection functions of the camera model by testing pixels on a grid
// in the entire image to ensure that the effects of lens distortion at the
// edges of the image are modeled correctly.
void ReprojectionTest(const DoubleSphereCameraModel& camera)
{
    constexpr double kTolerance = 1e-6;
    const double kNormalizedTolerance = kTolerance / camera.focalLength();
    constexpr int kImageWidth = 1280;
    constexpr int kImageHeight = 1024;
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
                    << "\nreprojected pixel: " << reprojected_pixel.transpose();
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
                Vector3d normalized_ray = camera.imageToSpace(pixel);
                normalized_ray /= normalized_ray[2];
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

TEST(DoubleSphereCameraModel, ReprojectionDistortion)
{
    constexpr double kPrincipalPoint[2] = {620.8633446632857,
                                           512.7479213012696};
    DoubleSphereCameraModel camera;
    camera.setFocalLength(364.4687315519798);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setDistortion(0.572501301952097, -0.2725048615813684);
    ReprojectionTest(camera);
}
