#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <tCamera/PinholeCameraModel>

using namespace tl;

using Eigen::Vector2d;
using Eigen::Vector3d;

TEST(PinholeCameraModel, InternalParameterGettersAndSetters)
{
    PinholeCameraModel camera;

    EXPECT_EQ(camera.type(), CameraIntrinsics::Type::Pinhole);

    // Check that default values are set
    EXPECT_EQ(camera.focalLength(), 1.0);
    EXPECT_EQ(camera.aspectRatio(), 1.0);
    EXPECT_EQ(camera.skew(), 0.0);
    EXPECT_EQ(camera.principalPointX(), 0.0);
    EXPECT_EQ(camera.principalPointY(), 0.0);
    EXPECT_EQ(camera.radialDistortion1(), 0.0);
    EXPECT_EQ(camera.radialDistortion2(), 0.0);

    // Set parameters to different values.
    camera.setFocalLength(600.0);
    camera.setAspectRatio(0.9);
    camera.setSkew(0.01);
    camera.setPrincipalPoint(300.0, 400.0);
    camera.setRadialDistortion(0.01, 0.001);

    // Check that the values were updated.
    EXPECT_EQ(camera.focalLength(), 600.0);
    EXPECT_EQ(camera.aspectRatio(), 0.9);
    EXPECT_EQ(camera.skew(), 0.01);
    EXPECT_EQ(camera.principalPointX(), 300.0);
    EXPECT_EQ(camera.principalPointY(), 400.0);
    EXPECT_EQ(camera.radialDistortion1(), 0.01);
    EXPECT_EQ(camera.radialDistortion2(), 0.001);
}

// Test to ensure that the camera intrinsics are being set appropriately.
inline void TestSetFromCameraMetaData(const CameraMetaData& prior)
{
    const PinholeCameraModel default_camera;
    PinholeCameraModel camera;
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
TEST(PinholeCameraModel, SetFromCameraMetaDatas)
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

TEST(PinholeCameraModel, constantParameterIndices)
{
    PinholeCameraModel camera;
    std::vector<int> constant_subset;

    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::None);
    EXPECT_EQ(constant_subset.size(), camera.numParameters());

    // Test that optimizing for focal length works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::FocalLength);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], PinholeCameraModel::Fx);
    }

    // Test that optimizing for principal_points works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::PrincipalPoint);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 2);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], PinholeCameraModel::Cx);
        EXPECT_NE(constant_subset[i], PinholeCameraModel::Cy);
    }

    // Test that optimizing for aspect ratio works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::AspectRatio);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], PinholeCameraModel::YX);
    }

    // Test that optimizing for skew works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::Skew);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], PinholeCameraModel::Skew);
    }

    // Test that optimizing for radial distortion works correctly.
    constant_subset = camera.constantParameterIndices(
        OptimizeIntrinsicsType::RadialDistortion);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 2);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], PinholeCameraModel::K1);
        EXPECT_NE(constant_subset[i], PinholeCameraModel::K2);
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
void ReprojectionTest(const PinholeCameraModel& camera)
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

TEST(PinholeCameraModel, ReprojectionNoDistortion)
{
    constexpr double kPrincipalPoint[2] = {600.0, 400.0};
    constexpr double kFocalLength = 1200;

    PinholeCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(0, 0);
    ReprojectionTest(camera);
}

TEST(PinholeCameraModel, ReprojectionOneDistortion)
{
    constexpr double kPrincipalPoint[2] = {600.0, 400.0};
    constexpr double kFocalLength = 1200;

    PinholeCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(0.01, 0);
    ReprojectionTest(camera);
}

TEST(PinholeCameraModel, ReprojectionTwoDistortion)
{
    constexpr double kPrincipalPoint[2] = {600.0, 400.0};
    constexpr double kFocalLength = 1200;

    PinholeCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(0.01, 0.001);
    ReprojectionTest(camera);
}

#include <tCamera/PinholeRadialTangentialCameraModel>

TEST(PinholeRadialTangentialCameraModel, InternalParameterGettersAndSetters)
{
    PinholeRadialTangentialCameraModel camera;

    EXPECT_EQ(camera.type(), CameraIntrinsics::Type::PinholeRadialTangential);

    // Check that default values are set
    EXPECT_EQ(camera.focalLength(), 1.0);
    EXPECT_EQ(camera.aspectRatio(), 1.0);
    EXPECT_EQ(camera.skew(), 0.0);
    EXPECT_EQ(camera.principalPointX(), 0.0);
    EXPECT_EQ(camera.principalPointY(), 0.0);
    EXPECT_EQ(camera.radialDistortion1(), 0.0);
    EXPECT_EQ(camera.radialDistortion2(), 0.0);
    EXPECT_EQ(camera.radialDistortion3(), 0.0);
    EXPECT_EQ(camera.tangentialDistortion1(), 0.0);
    EXPECT_EQ(camera.tangentialDistortion2(), 0.0);

    // Set parameters to different values.
    camera.setFocalLength(600.0);
    camera.setAspectRatio(0.9);
    camera.setSkew(0.01);
    camera.setPrincipalPoint(300.0, 400.0);
    camera.setRadialDistortion(0.01, 0.001, 0.0001);
    camera.setTangentialDistortion(0.02, 0.002);

    // Check that the values were updated.
    EXPECT_EQ(camera.focalLength(), 600.0);
    EXPECT_EQ(camera.aspectRatio(), 0.9);
    EXPECT_EQ(camera.skew(), 0.01);
    EXPECT_EQ(camera.principalPointX(), 300.0);
    EXPECT_EQ(camera.principalPointY(), 400.0);
    EXPECT_EQ(camera.radialDistortion1(), 0.01);
    EXPECT_EQ(camera.radialDistortion2(), 0.001);
    EXPECT_EQ(camera.radialDistortion3(), 0.0001);
    EXPECT_EQ(camera.tangentialDistortion1(), 0.02);
    EXPECT_EQ(camera.tangentialDistortion2(), 0.002);
}

// Test to ensure that the camera intrinsics are being set appropriately.
inline void TestSetFromCameraintrinsicsPrior(const CameraMetaData& prior)
{
    const PinholeRadialTangentialCameraModel default_camera;

    PinholeRadialTangentialCameraModel camera;
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
TEST(PinholeRadialTangentialCameraModel, SetFromCameraIntrinsicsPriors)
{
    CameraMetaData prior;
    prior.focal_length.value[0] = 1000.0;
    prior.principal_point.value[0] = 400.0;
    prior.principal_point.value[1] = 300.0;
    prior.aspect_ratio.value[0] = 1.01;
    prior.skew.value[0] = 0.01;
    prior.radial_distortion.value[0] = 0.1;
    prior.radial_distortion.value[1] = 0.01;
    prior.radial_distortion.value[2] = 0.001;
    prior.tangential_distortion.value[0] = 0.1;
    prior.tangential_distortion.value[1] = 0.01;

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

    prior.tangential_distortion.is_set = true;
    TestSetFromCameraintrinsicsPrior(prior);
}

TEST(PinholeRadialTangentialCameraModel, GetSubsetFromOptimizeIntrinsicsType)
{
    PinholeRadialTangentialCameraModel camera;
    std::vector<int> constant_subset;

    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::None);
    EXPECT_EQ(constant_subset.size(), camera.numParameters());

    // Test that optimizing for focal length works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::FocalLength);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], PinholeRadialTangentialCameraModel::Fx);
    }

    // Test that optimizing for principal_points works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::PrincipalPoint);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 2);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], PinholeRadialTangentialCameraModel::Cx);
        EXPECT_NE(constant_subset[i], PinholeRadialTangentialCameraModel::Cy);
    }

    // Test that optimizing for aspect ratio works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::AspectRatio);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], PinholeRadialTangentialCameraModel::YX);
    }

    // Test that optimizing for skew works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::Skew);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], PinholeRadialTangentialCameraModel::Skew);
    }

    // Test that optimizing for radial distortion works correctly.
    constant_subset = camera.constantParameterIndices(
        OptimizeIntrinsicsType::RadialDistortion);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 3);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], PinholeRadialTangentialCameraModel::K1);
        EXPECT_NE(constant_subset[i], PinholeRadialTangentialCameraModel::K2);
        EXPECT_NE(constant_subset[i], PinholeRadialTangentialCameraModel::K3);
    }

    // Test that optimizing for tangential distortion does not optimize any
    // parameters.
    constant_subset = camera.constantParameterIndices(
        OptimizeIntrinsicsType::TangentialDistortion);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 2);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], PinholeRadialTangentialCameraModel::T1);
        EXPECT_NE(constant_subset[i], PinholeRadialTangentialCameraModel::T2);
    }
}

void ReprojectionTest(const PinholeRadialTangentialCameraModel& camera)
{
    constexpr double kTolerance = 1e-5;
    const double kNormalizedTolerance = kTolerance / camera.focalLength();
    constexpr int kImageWidth = 1200;
    constexpr int kImageHeight = 980;
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

TEST(PinholeRadialTangentialCameraModel, ReprojectionNoDistortion)
{
    constexpr double kPrincipalPoint[2]{600., 400.};
    constexpr double kFocalLength{1200.};

    PinholeRadialTangentialCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(0., 0., 0.);
    ReprojectionTest(camera);
}

TEST(PinholeRadialTangentialCameraModel, ReprojectionOneRadialDistortion)
{
    constexpr double kPrincipalPoint[2]{600., 400.};
    constexpr double kFocalLength{1200.};

    PinholeRadialTangentialCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(0.01, 0., 0.);
    ReprojectionTest(camera);
}

TEST(PinholeRadialTangentialCameraModel, ReprojectionTwoRadialDistortion)
{
    constexpr double kPrincipalPoint[2]{600., 400.};
    constexpr double kFocalLength{1200.};

    PinholeRadialTangentialCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(0.01, 0.001, 0.);
    ReprojectionTest(camera);
}

TEST(PinholeRadialTangentialCameraModel, ReprojectionOneTangentialDistortion)
{
    constexpr double kPrincipalPoint[2]{600., 400.};
    constexpr double kFocalLength{1200.};

    PinholeRadialTangentialCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(0.01, 0.001, 0.0001);
    camera.setTangentialDistortion(0.01, 0.);
    ReprojectionTest(camera);
}

TEST(PinholeRadialTangentialCameraModel, ReprojectionTwoTangentialDistortion)
{
    constexpr double kPrincipalPoint[2]{600., 400.};
    constexpr double kFocalLength{1200.};

    PinholeRadialTangentialCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(0.01, 0.001, 0.0001);
    camera.setTangentialDistortion(0.01, 0.001);
    ReprojectionTest(camera);
}
