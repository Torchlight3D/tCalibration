#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <tCamera/Camera>
#include <tCamera/DivisionUndistortionCameraModel>
#include <tCore/Math>

using namespace tl;

using Eigen::Vector2d;
using Eigen::Vector3d;

TEST(DivisionUndistortionCameraModel, InternalParameterGettersAndSetters)
{
    DivisionUndistortionCameraModel camera;

    EXPECT_EQ(camera.type(), CameraIntrinsics::Type::DivisionUndistortion);

    // Check that default values are set
    EXPECT_EQ(camera.focalLength(), 1.0);
    EXPECT_EQ(camera.aspectRatio(), 1.0);
    EXPECT_EQ(camera.principalPointX(), 0.0);
    EXPECT_EQ(camera.principalPointY(), 0.0);
    EXPECT_EQ(camera.radialDistortion1(), 0.0);

    // Set parameters to different values.
    camera.setFocalLength(600.0);
    camera.setAspectRatio(0.9);
    camera.setPrincipalPoint(300.0, 400.0);
    camera.setRadialDistortion(-0.01);

    // Check that the values were updated.
    EXPECT_EQ(camera.focalLength(), 600.0);
    EXPECT_EQ(camera.aspectRatio(), 0.9);
    EXPECT_EQ(camera.principalPointX(), 300.0);
    EXPECT_EQ(camera.principalPointY(), 400.0);
    EXPECT_EQ(camera.radialDistortion1(), -0.01);
}

TEST(DivisionUndistortionCameraModel, CameraParameterGettersAndSetters)
{
    Camera camera(CameraIntrinsics::Type::DivisionUndistortion);

    EXPECT_EQ(camera.cameraIntrinsics()->type(),
              CameraIntrinsics::Type::DivisionUndistortion);

    // Check that default values are set
    EXPECT_EQ(camera.focalLength(), 1.0);
    EXPECT_EQ(camera.principalPointX(), 0.0);
    EXPECT_EQ(camera.principalPointY(), 0.0);

    // Set parameters to different values.
    camera.setFocalLength(600.0);
    camera.setPrincipalPoint(300.0, 400.0);

    // Check that the values were updated.
    EXPECT_EQ(camera.focalLength(), 600.0);
    EXPECT_EQ(camera.principalPointX(), 300.0);
    EXPECT_EQ(camera.principalPointY(), 400.0);
}

// Test to ensure that the camera intrinsics are being set appropriately.
inline void TestSetFromCameraMetaData(const CameraMetaData& prior)
{
    const DivisionUndistortionCameraModel default_camera;
    DivisionUndistortionCameraModel camera;
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
TEST(DivisionUndistortionCameraModel, SetFromCameraMetaDatas)
{
    CameraMetaData prior;
    prior.focal_length.value[0] = 1000.0;
    prior.principal_point.value[0] = 400.0;
    prior.principal_point.value[1] = 300.0;
    prior.aspect_ratio.value[0] = 1.01;
    prior.radial_distortion.value[0] = -0.01;

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

TEST(DivisionUndistortionCameraModel, GetSubsetFromOptimizeIntrinsicsType)
{
    DivisionUndistortionCameraModel camera;
    std::vector<int> constant_subset;

    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::None);
    EXPECT_EQ(constant_subset.size(), camera.numParameters());

    // Test that optimizing for focal length works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::FocalLength);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], DivisionUndistortionCameraModel::Fx);
    }

    // Test that optimizing for principal_points works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::PrincipalPoint);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 2);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], DivisionUndistortionCameraModel::Cx);
        EXPECT_NE(constant_subset[i], DivisionUndistortionCameraModel::Cy);
    }

    // Test that optimizing for aspect ratio works correctly.
    constant_subset =
        camera.constantParameterIndices(OptimizeIntrinsicsType::AspectRatio);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], DivisionUndistortionCameraModel::YX);
    }

    // Test that optimizing for radial distortion works correctly.
    constant_subset = camera.constantParameterIndices(
        OptimizeIntrinsicsType::RadialDistortion);
    EXPECT_EQ(constant_subset.size(), camera.numParameters() - 1);
    for (int i = 0; i < constant_subset.size(); i++) {
        EXPECT_NE(constant_subset[i], DivisionUndistortionCameraModel::K);
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

inline void DistortionTest(const DivisionUndistortionCameraModel& camera)
{
    constexpr double kTolerance = 1e-8;
    const double kNormalizedTolerance = kTolerance / camera.focalLength();
    constexpr int kImageWidth = 1200;
    constexpr int kImageHeight = 800;
    constexpr double kMinDepth = 2.0;
    constexpr double kMaxDepth = 25.0;

    // Ensure the distored -> undistorted -> distorted transformation works.
    for (double x = 0.0; x < kImageWidth; x += 10.0) {
        for (double y = 0.0; y < kImageHeight; y += 10.0) {
            const Eigen::Vector2d distorted_pixel(x - camera.principalPointX(),
                                                  y - camera.principalPointY());

            Eigen::Vector2d undistorted_pixel;
            DivisionUndistortionCameraModel::undistort(
                camera.parameters(), distorted_pixel.data(),
                undistorted_pixel.data());

            Eigen::Vector2d redistorted_pixel;
            DivisionUndistortionCameraModel::distort(camera.parameters(),
                                                     undistorted_pixel.data(),
                                                     redistorted_pixel.data());

            // Expect the original and redistorted pixel to be close.
            ASSERT_LT((distorted_pixel - redistorted_pixel).norm(), kTolerance)
                << "gt pixel: " << distorted_pixel.transpose()
                << "\nundistorted_pixel: " << undistorted_pixel.transpose()
                << "\nredistorted pixel: " << redistorted_pixel.transpose();
        }
    }

    // Ensure the undistorted -> distorted -> undistorted transformation works.
    for (double x = 0.0; x < kImageWidth; x += 10.0) {
        for (double y = 0.0; y < kImageHeight; y += 10.0) {
            // for (double x = -1.0; x < 1.0; x += 0.1) {
            //   for (double y = -1.0; y < 1.0; y += 0.1) {
            const Eigen::Vector2d undistorted_pixel(
                x - camera.principalPointX(), y - camera.principalPointY());

            Eigen::Vector2d distorted_pixel;
            DivisionUndistortionCameraModel::distort(camera.parameters(),
                                                     undistorted_pixel.data(),
                                                     distorted_pixel.data());

            Eigen::Vector2d reundistorted_pixel;
            DivisionUndistortionCameraModel::undistort(
                camera.parameters(), distorted_pixel.data(),
                reundistorted_pixel.data());

            // Expect the original and reundistorted pixel to be close.
            ASSERT_LT((undistorted_pixel - reundistorted_pixel).norm(),
                      kTolerance)
                << "gt pixel: " << undistorted_pixel.transpose()
                << "\ndistorted_pixel: " << distorted_pixel.transpose()
                << "\nreundistorted pixel: " << reundistorted_pixel.transpose();
        }
    }
}

TEST(DivisionUndistortionCameraModel, DistortionTestNoDistortion)
{
    constexpr double kPrincipalPoint[2] = {600.0, 400.0};
    constexpr double kFocalLength = 1200;
    DivisionUndistortionCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(0);
    DistortionTest(camera);
}

TEST(DivisionUndistortionCameraModel, DistortionTestSmall)
{
    constexpr double kPrincipalPoint[2] = {600.0, 400.0};
    constexpr double kFocalLength = 1200.0;
    DivisionUndistortionCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(-1e-8);
    DistortionTest(camera);
}

TEST(DivisionUndistortionCameraModel, DistortionTestMedium)
{
    constexpr double kPrincipalPoint[2] = {600.0, 400.0};
    constexpr double kFocalLength = 1200.0;
    DivisionUndistortionCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);

    camera.setRadialDistortion(-1e-7);
    DistortionTest(camera);
}

TEST(DivisionUndistortionCameraModel, DistortionTestLarge)
{
    constexpr double kPrincipalPoint[2] = {600.0, 400.0};
    constexpr double kFocalLength = 1200.0;
    DivisionUndistortionCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);

    camera.setRadialDistortion(-1e-6);
    DistortionTest(camera);
}

void ReprojectionTest(const DivisionUndistortionCameraModel& camera)
{
    constexpr double kTolerance = 1e-6;
    const double kNormalizedTolerance = kTolerance / camera.focalLength();
    constexpr int kImageWidth = 1200;
    constexpr int kImageHeight = 800;
    constexpr double kMinDepth = 2.0;
    constexpr double kMaxDepth = 25.0;

    // Ensure the Image -> Camera -> Image transformation works.
    {
        double x{0.}, y{0.};
        while (x < kImageWidth) {
            while (y < kImageHeight) {
                // for (double x = -1.0; x < 1.0; x += 0.1) {
                //   for (double y = -1.0; y < 1.0; y += 0.1) {
                const Eigen::Vector2d pixel(x, y);
                // Get the normalized ray of that pixel.
                const Vector3d normalized_ray = camera.imageToSpace(pixel);

                // Test the reprojection at several depths.
                double depth{kMinDepth};
                while (depth < kMaxDepth) {
                    // Convert it to a full 3D point in the camera coordinate
                    // system.
                    const Vector3d point = normalized_ray * depth;
                    const Vector2d reprojected_pixel =
                        camera.spaceToImage(point);

                    // Expect the reprojection to be close.
                    ASSERT_LT((pixel - reprojected_pixel).norm(), kTolerance)
                        << "gt pixel: " << pixel.transpose()
                        << "\nreprojected pixel: "
                        << reprojected_pixel.transpose();

                    depth += 1.0;
                }

                y += 10.;
            }
            x += 10.;
        }
    }

    // Ensure the Camera -> Image -> Camera transformation works.
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
                ASSERT_LT((point - reprojected_point).norm(),
                          kNormalizedTolerance)
                    << "gt pixel: " << point.transpose()
                    << "\nreprojected pixel: " << reprojected_point.transpose();
            }
        }
    }
}

TEST(DivisionUndistortionCameraModel, ReprojectionNoDistortion)
{
    constexpr double kPrincipalPoint[2] = {600.0, 400.0};
    constexpr double kFocalLength = 1200;
    DivisionUndistortionCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(0);
    ReprojectionTest(camera);
}

TEST(DivisionUndistortionCameraModel, ReprojectionSmall)
{
    constexpr double kPrincipalPoint[2] = {600.0, 400.0};
    constexpr double kFocalLength = 1200.0;
    DivisionUndistortionCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    camera.setRadialDistortion(-1e-8);
    ReprojectionTest(camera);
}

TEST(DivisionUndistortionCameraModel, ReprojectionMedium)
{
    constexpr double kPrincipalPoint[2] = {600.0, 400.0};
    constexpr double kFocalLength = 1200;
    DivisionUndistortionCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);

    camera.setRadialDistortion(-1e-7);
    ReprojectionTest(camera);
}

TEST(DivisionUndistortionCameraModel, ReprojectionLarge)
{
    constexpr double kPrincipalPoint[2] = {600.0, 400.0};
    constexpr double kFocalLength = 1200;
    DivisionUndistortionCameraModel camera;
    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);

    camera.setRadialDistortion(-1e-6);
    ReprojectionTest(camera);
}

TEST(DivisionUndistortionCameraModel, Triangulation)
{
    constexpr double kFocalLength = 3587.6;
    constexpr double kUndistortion = -1.07574e-08;
    const Eigen::Vector2d kPrincipalPoint(1980.0, 1200.0);
    const Eigen::Vector4d kPoint(-2.3, 1.7, 6., 1.);

    Camera camera1(CameraIntrinsics::Type::DivisionUndistortion);
    camera1.setFocalLength(kFocalLength);
    camera1.setPrincipalPoint(kPrincipalPoint.x(), kPrincipalPoint.y());
    camera1.rIntrinsics()[DivisionUndistortionCameraModel::K] = kUndistortion;
    Camera camera2 = camera1;
    camera2.setOrientationFromAngleAxis(Eigen::Vector3d(-0.1, -0.4, 0.3));
    camera2.setPosition(Eigen::Vector3d(0.8, 0.2, 0.1));

    Eigen::Vector2d feature1, feature2;
    const double depth1 = camera1.projectPoint(kPoint, feature1);
    const double depth2 = camera2.projectPoint(kPoint, feature2);
    CHECK_GT(depth1, 0.0);
    CHECK_GT(depth2, 0.0);
    // Maybe check features too?

    const Eigen::Vector3d gt_ray1 =
        (kPoint.hnormalized() - camera1.position()).normalized();
    const Eigen::Vector3d gt_ray2 =
        (kPoint.hnormalized() - camera2.position()).normalized();
    const Eigen::Vector3d ray1 =
        camera1.pixelToUnitDepthRay(feature1).normalized();
    const Eigen::Vector3d ray2 =
        camera2.pixelToUnitDepthRay(feature2).normalized();

    const double angle1 = math::radToDeg(std::acos(gt_ray1.dot(ray1)));
    const double angle2 = math::radToDeg(std::acos(gt_ray2.dot(ray2)));
    CHECK_LT(std::abs(angle1), 1e-6);
    CHECK_LT(std::abs(angle1), 1e-6);
}

TEST(DivisionUndistortionCameraModel, NoDistortion)
{
    const Eigen::Vector4d point(-2.3, 1.7, 6, 1.0);
    const double kFocalLength = 3587.6;
    const Eigen::Vector2d kPrincipalPoint(1980.0, 1200.0);
    Camera camera1(CameraIntrinsics::Type::DivisionUndistortion);
    camera1.setFocalLength(kFocalLength);
    camera1.setPrincipalPoint(kPrincipalPoint.x(), kPrincipalPoint.y());
    camera1.setOrientationFromAngleAxis(Eigen::Vector3d(-0.1, -0.4, 0.3));
    camera1.setPosition(Eigen::Vector3d(0.8, 0.2, 0.1));

    Camera camera2(CameraIntrinsics::Type::Pinhole);
    camera2.setOrientationFromAngleAxis(Eigen::Vector3d(-0.1, -0.4, 0.3));
    camera2.setPosition(Eigen::Vector3d(0.8, 0.2, 0.1));
    camera2.setFocalLength(kFocalLength);
    camera2.setPrincipalPoint(kPrincipalPoint.x(), kPrincipalPoint.y());

    Eigen::Vector2d feature1, feature2;
    const double depth1 = camera1.projectPoint(point, feature1);
    const double depth2 = camera2.projectPoint(point, feature2);
    EXPECT_DOUBLE_EQ(depth1, depth2);
    EXPECT_DOUBLE_EQ(feature1.x(), feature2.x());
    EXPECT_DOUBLE_EQ(feature1.y(), feature2.y());

    Eigen::Vector2d undistorted_pixel;
    DivisionUndistortionCameraModel::DistortedPixelToUndistortedPixel(
        camera1.intrinsics(), feature1.data(), undistorted_pixel.data());
    EXPECT_DOUBLE_EQ(feature1.x(), undistorted_pixel.x());
    EXPECT_DOUBLE_EQ(feature1.y(), undistorted_pixel.y());
}
