#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <tCamera/FovCameraModel>

using namespace tl;

using Eigen::Vector2d;
using Eigen::Vector3d;

TEST(FOVCameraModel, InternalParameterGettersAndSetters)
{
    FOVCameraModel camera;
    constexpr double kDefaultOmega{0.75};

    EXPECT_EQ(camera.type(), CameraIntrinsics::Type::Fov);

    // Check that default values are set
    EXPECT_EQ(camera.focalLength(), 1.0);
    EXPECT_EQ(camera.aspectRatio(), 1.0);
    EXPECT_EQ(camera.principalPointX(), 0.0);
    EXPECT_EQ(camera.principalPointY(), 0.0);
    EXPECT_EQ(camera.radialDistortion1(), kDefaultOmega);

    // Set parameters to different values.
    camera.setFocalLength(600.0);
    camera.setAspectRatio(0.9);
    camera.setPrincipalPoint(300.0, 400.0);
    camera.setRadialDistortion(0.01);

    // Check that the values were updated.
    EXPECT_EQ(camera.focalLength(), 600.0);
    EXPECT_EQ(camera.aspectRatio(), 0.9);
    EXPECT_EQ(camera.principalPointX(), 300.0);
    EXPECT_EQ(camera.principalPointY(), 400.0);
    EXPECT_EQ(camera.radialDistortion1(), 0.01);
}

// Test to ensure that the camera intrinsics are being set appropriately.
inline void TestSetFromCameraMetaData(const CameraMetaData& prior)
{
    const FOVCameraModel default_camera;
    FOVCameraModel camera;
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

    if (prior.radial_distortion.is_set) {
        EXPECT_EQ(camera.radialDistortion1(), prior.radial_distortion.value[0]);
    }
    else {
        EXPECT_EQ(camera.radialDistortion1(),
                  default_camera.radialDistortion1());
    }
}

// Gradually add one prior at a time and ensure that the method still works. We
// test before and after setting the "is_set" member variable to true to ensure
// that setting the value of priors when is_set=false is handled properly.
TEST(FOVCameraModel, SetFromCameraMetaDatas)
{
    CameraMetaData prior;
    prior.focal_length.value[0] = 1000.0;
    prior.principal_point.value[0] = 400.0;
    prior.principal_point.value[1] = 300.0;
    prior.aspect_ratio.value[0] = 1.01;
    prior.radial_distortion.value[0] = 0.01;

    TestSetFromCameraMetaData(prior);

    prior.focal_length.is_set = true;
    TestSetFromCameraMetaData(prior);

    prior.principal_point.is_set = true;
    TestSetFromCameraMetaData(prior);

    prior.aspect_ratio.is_set = true;
    TestSetFromCameraMetaData(prior);

    prior.radial_distortion.is_set = true;
    TestSetFromCameraMetaData(prior);
}

TEST(FOVCameraModel, constantParameterIndices)
{
    FOVCameraModel camera;
    std::vector<int> constant_subset;

    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::None);
    EXPECT_EQ(constant_subset.size(), camera.numParameters());

    // Test that optimizing for focal length works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::FocalLength);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], FOVCameraModel::Fx);
    }

    // Test that optimizing for principal_points works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::PrincipalPoint);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 2);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], FOVCameraModel::Cx);
        EXPECT_NE(constant_subset[i], FOVCameraModel::Cy);
    }

    // Test that optimizing for aspect ratio works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::AspectRatio);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], FOVCameraModel::YX);
    }

    // Test that optimizing for radial distortion works correctly.
    constant_subset = camera.constantParameterIndices(
        OptimizeIntrinsicsType::RadialDistortion);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], FOVCameraModel::Omega);
    }

    // Test that optimizing for skew and tangential distortion does not optimize
    // any parameters.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::Skew);
    EXPECT_EQ(constant_subset.size(), camera.numParameters());
    constant_subset = camera.constantParameterIndices(
        OptimizeIntrinsicsType::TangentialDistortion);
    EXPECT_EQ(constant_subset.size(), camera.numParameters());
}

void ReprojectionTest(const FOVCameraModel& camera)
{
    constexpr double kTolerance{1e-5};
    const double kNormalizedTolerance = kTolerance / camera.focalLength();
    constexpr int kImageWidth{1200};
    constexpr int kImageHeight{980};
    constexpr double kMinDepth{2.0};
    constexpr double kMaxDepth{25.0};

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

TEST(FOVCameraModel, ReprojectionNoDistortion)
{
    constexpr double kPrincipalPoint[2]{600., 400.};
    constexpr double kFocalLength{1200.};
    FOVCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(0);
    ReprojectionTest(camera);
}

TEST(FOVCameraModel, ReprojectionOneDistortionSmall)
{
    constexpr double kPrincipalPoint[2]{600., 400.};
    constexpr double kFocalLength{1200.};
    FOVCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);

    camera.setRadialDistortion(0.0001);
    ReprojectionTest(camera);
}

TEST(FOVCameraModel, ReprojectionOneDistortionMedium)
{
    constexpr double kPrincipalPoint[2]{600., 400.};
    constexpr double kFocalLength{1200.};
    FOVCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);

    camera.setRadialDistortion(0.001);
    ReprojectionTest(camera);
}

TEST(FOVCameraModel, ReprojectionOneDistortionLarge)
{
    constexpr double kPrincipalPoint[2]{600., 400.};
    constexpr double kFocalLength{1200.};
    FOVCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);

    camera.setRadialDistortion(0.1);
    ReprojectionTest(camera);
}
