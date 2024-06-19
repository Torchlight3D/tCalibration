#include <Eigen/Core>
#include <gtest/gtest.h>

#include <tCamera/Camera>
#include <tCamera/DivisionUndistortionCameraModel>
#include <tCore/Math>

using namespace tl;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

TEST(DivisionUndistortionCameraModel, CreateFromName)
{
    using theFactory = ::factory::Registry<CameraIntrinsics>;

    ASSERT_TRUE(theFactory::CanNew(DivisionUndistortionCameraModel::kName));
    ASSERT_TRUE(theFactory::New(DivisionUndistortionCameraModel::kName).get());
}

TEST(DivisionUndistortionCameraModel, ParameterGettersAndSetters)
{
    DivisionUndistortionCameraModel camera;

    // Check type
    EXPECT_EQ(camera.type(), CameraIntrinsicsType::DivisionUndistortion);

    // Check that default values are set
    EXPECT_EQ(camera.focalLength(), 1.);
    EXPECT_EQ(camera.aspectRatio(), 1.);
    EXPECT_EQ(camera.principalPointX(), 0.);
    EXPECT_EQ(camera.principalPointY(), 0.);
    EXPECT_EQ(camera.radialDistortion1(), 0.);

    // Set parameters to different values, and check that they were updated.
    constexpr auto kFocalLength{600.};
    constexpr auto kAspectRatio{0.9};
    constexpr auto kPrincipalPointX{300.};
    constexpr auto kPrincipalPointY{400.};
    constexpr auto kRadialDistortion{-0.01};

    camera.setFocalLength(kFocalLength);
    camera.setAspectRatio(kAspectRatio);
    camera.setPrincipalPoint(kPrincipalPointX, kPrincipalPointY);
    camera.setRadialDistortion(kRadialDistortion);

    EXPECT_EQ(camera.focalLength(), kFocalLength);
    EXPECT_EQ(camera.aspectRatio(), kAspectRatio);
    EXPECT_EQ(camera.principalPointX(), kPrincipalPointX);
    EXPECT_EQ(camera.principalPointY(), kPrincipalPointY);
    EXPECT_EQ(camera.radialDistortion1(), kRadialDistortion);
}

TEST(DivisionUndistortionCameraModel, CameraParameterGettersAndSetters)
{
    Camera camera{CameraIntrinsicsType::DivisionUndistortion};

    EXPECT_EQ(camera.cameraIntrinsics()->type(),
              CameraIntrinsicsType::DivisionUndistortion);

    // Check that default values are set
    EXPECT_EQ(camera.focalLength(), 1.0);
    EXPECT_EQ(camera.principalPointX(), 0.0);
    EXPECT_EQ(camera.principalPointY(), 0.0);

    // Set parameters to different values, and check that they were updated.
    constexpr auto kFocalLength{600.};
    constexpr auto kPrincipalPointX{300.};
    constexpr auto kPrincipalPointY{400.};

    camera.setFocalLength(kFocalLength);
    camera.setPrincipalPoint(kPrincipalPointX, kPrincipalPointY);

    EXPECT_EQ(camera.focalLength(), kFocalLength);
    EXPECT_EQ(camera.principalPointX(), kPrincipalPointX);
    EXPECT_EQ(camera.principalPointY(), kPrincipalPointY);
}

TEST(DivisionUndistortionCameraModel, SetFromCameraMetaData)
{
    auto TestCameraSetFromMeta = [](const CameraMetaData& meta) {
        const DivisionUndistortionCameraModel default_camera;

        DivisionUndistortionCameraModel camera;
        camera.setFromMetaData(meta);

        if (meta.focalLength.has_value()) {
            EXPECT_EQ(camera.focalLength(), meta.focalLength.value()[0]);
        }
        else {
            EXPECT_EQ(camera.focalLength(), default_camera.focalLength());
        }

        if (meta.aspectRatio.has_value()) {
            EXPECT_EQ(camera.aspectRatio(), meta.aspectRatio.value()[0]);
        }
        else {
            EXPECT_EQ(camera.aspectRatio(), default_camera.aspectRatio());
        }

        if (meta.principalPoint.has_value()) {
            EXPECT_EQ(camera.cx(), meta.cx());
            EXPECT_EQ(camera.cy(), meta.cy());
        }
        else {
            EXPECT_EQ(camera.cx(), default_camera.cx());
            EXPECT_EQ(camera.cy(), default_camera.cy());
        }

        if (meta.radialDistortion.has_value()) {
            EXPECT_EQ(camera.k(), meta.radialDistortion.value()[0]);
        }
        else {
            EXPECT_EQ(camera.k(), default_camera.k());
        }
    };

    CameraMetaData meta;
    TestCameraSetFromMeta(meta);

    meta.focalLength = {1000.};
    TestCameraSetFromMeta(meta);

    meta.aspectRatio = {1.01};
    TestCameraSetFromMeta(meta);

    meta.principalPoint = {400., 300.};
    TestCameraSetFromMeta(meta);

    meta.radialDistortion = {-0.01};
    TestCameraSetFromMeta(meta);
}

TEST(DivisionUndistortionCameraModel, ConstantParameterIndices)
{
    using _Type = OptimizeIntrinsicsType;

    DivisionUndistortionCameraModel camera;
    std::vector<int> indices;

    indices = camera.fixedParameterIndices(_Type::None);
    EXPECT_EQ(indices.size(), camera.numParameters());

    // Focal length
    indices = camera.fixedParameterIndices(_Type::FocalLength);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, DivisionUndistortionCameraModel::Fx);
    }

    // Principal point
    indices = camera.fixedParameterIndices(_Type::PrincipalPoint);
    EXPECT_EQ(indices.size(), camera.numParameters() - 2);
    for (const auto& index : indices) {
        EXPECT_NE(index, DivisionUndistortionCameraModel::Cx);
        EXPECT_NE(index, DivisionUndistortionCameraModel::Cy);
    }

    // Aspect ratio
    indices = camera.fixedParameterIndices(_Type::AspectRatio);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, DivisionUndistortionCameraModel::YX);
    }

    // Radial distortion
    indices = camera.fixedParameterIndices(_Type::RadialDistortion);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, DivisionUndistortionCameraModel::K);
    }

    // Skew and tangential distortion
    indices = camera.fixedParameterIndices(_Type::Skew);
    EXPECT_EQ(indices.size(), camera.numParameters());
    indices = camera.fixedParameterIndices(_Type::TangentialDistortion);
    EXPECT_EQ(indices.size(), camera.numParameters());
}

class DivisionUndistortionCameraFixture : public ::testing::Test
{
protected:
    void SetUp() override
    {
        constexpr double kPrincipalPoint[2] = {600., 400.};
        constexpr double kFocalLength = 1200.;

        _camera.setFocalLength(kFocalLength);
        _camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    }

    void TestDistortion() const
    {
        constexpr double kTolerance = 1e-8;
        const double kNormalizedTolerance = kTolerance / _camera.focalLength();
        constexpr int kImageWidth = 1200;
        constexpr int kImageHeight = 800;
        constexpr double kMinDepth = 2.0;
        constexpr double kMaxDepth = 25.0;

        // Ensure the distored -> undistorted -> distorted transformation works.
        for (double x = 0.0; x < kImageWidth; x += 10.0) {
            for (double y = 0.0; y < kImageHeight; y += 10.0) {
                const Vector2d px_d{x - _camera.cx(), y - _camera.cy()};

                Vector2d px_u;
                DivisionUndistortionCameraModel::undistortPoint(
                    _camera.parameters(), px_d.data(), px_u.data());

                Vector2d redistorted_pixel;
                DivisionUndistortionCameraModel::distortPoint(
                    _camera.parameters(), px_u.data(),
                    redistorted_pixel.data());

                // Expect the original and redistorted pixel to be close.
                ASSERT_LT((px_d - redistorted_pixel).norm(), kTolerance)
                    << "GT pixel: " << px_d.transpose()
                    << "\n"
                       "Undistorted pixel: "
                    << px_u.transpose()
                    << "\n"
                       "Redistorted pixel: "
                    << redistorted_pixel.transpose();
            }
        }

        // Ensure the undistorted -> distorted -> undistorted transformation
        // works.
        for (double x = 0.0; x < kImageWidth; x += 10.0) {
            for (double y = 0.0; y < kImageHeight; y += 10.0) {
                // for (double x = -1.0; x < 1.0; x += 0.1) {
                //   for (double y = -1.0; y < 1.0; y += 0.1) {
                const Vector2d undistorted_pixel(x - _camera.cx(),
                                                 y - _camera.cy());

                Vector2d distorted_pixel;
                DivisionUndistortionCameraModel::undistortPoint(
                    _camera.parameters(), undistorted_pixel.data(),
                    distorted_pixel.data());

                Vector2d reundistorted_pixel;
                DivisionUndistortionCameraModel::distortPoint(
                    _camera.parameters(), distorted_pixel.data(),
                    reundistorted_pixel.data());

                // Expect the original and reundistorted pixel to be close.
                ASSERT_LT((undistorted_pixel - reundistorted_pixel).norm(),
                          kTolerance)
                    << "GT pixel: " << undistorted_pixel.transpose()
                    << "\n"
                       "Distorted pixel: "
                    << distorted_pixel.transpose()
                    << "\n"
                       "Reundistorted pixel: "
                    << reundistorted_pixel.transpose();
            }
        }
    }

    void TestProjection()
    {
        constexpr double kTolerance = 1e-6;
        const double kNormalizedTolerance = kTolerance / _camera.focalLength();
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
                    const Vector2d pixel(x, y);
                    // Get the normalized ray of that pixel.
                    const Vector3d normalized_ray = _camera.imageToSpace(pixel);

                    // Test the reprojection at several depths.
                    double depth{kMinDepth};
                    while (depth < kMaxDepth) {
                        // Convert it to a full 3D point in the camera
                        // coordinate system.
                        const Vector3d point = normalized_ray * depth;
                        const Vector2d reprojected_pixel =
                            _camera.spaceToImage(point);

                        // Expect the reprojection to be close.
                        ASSERT_LT((pixel - reprojected_pixel).norm(),
                                  kTolerance)
                            << "GT pixel: " << pixel.transpose()
                            << "\n"
                               "Reprojected pixel: "
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
                for (double depth = kMinDepth; depth < kMaxDepth; depth += 1.) {
                    const Vector3d point(x, y, depth);
                    const Vector2d pixel = _camera.spaceToImage(point);
                    const Vector3d normalized_ray = _camera.imageToSpace(pixel);
                    const Vector3d reprojected_point = normalized_ray * depth;

                    // Expect the reprojection to be close.
                    ASSERT_LT((point - reprojected_point).norm(),
                              kNormalizedTolerance)
                        << "GT pixel: " << point.transpose()
                        << "\n"
                           "Reprojected pixel: "
                        << reprojected_point.transpose();
                }
            }
        }
    }

protected:
    DivisionUndistortionCameraModel _camera;
};

TEST_F(DivisionUndistortionCameraFixture, NoDistortion)
{
    _camera.setRadialDistortion(0.);
    TestDistortion();
}

TEST_F(DivisionUndistortionCameraFixture, SmallDistortion)
{
    _camera.setRadialDistortion(-1e-8);
    TestDistortion();
}

TEST_F(DivisionUndistortionCameraFixture, MediumDistortion)
{
    _camera.setRadialDistortion(-1e-7);
    TestDistortion();
}

TEST_F(DivisionUndistortionCameraFixture, LargeDistortion)
{
    _camera.setRadialDistortion(-1e-6);
    TestDistortion();
}

TEST_F(DivisionUndistortionCameraFixture, ReprojectionNoDistortion)
{
    _camera.setRadialDistortion(0.);
    TestProjection();
}

TEST_F(DivisionUndistortionCameraFixture, ReprojectionSmallDistortion)
{
    _camera.setRadialDistortion(-1e-8);
    TestProjection();
}

TEST_F(DivisionUndistortionCameraFixture, ReprojectionMediumDistortion)
{
    _camera.setRadialDistortion(-1e-7);
    TestProjection();
}

TEST_F(DivisionUndistortionCameraFixture, ReprojectionLarge)
{
    _camera.setRadialDistortion(-1e-6);
    TestProjection();
}

TEST(DivisionUndistortionCameraModel, Triangulation)
{
    constexpr double kFocalLength = 3587.6;
    constexpr double kUndistortion = -1.07574e-08;
    const Vector2d kPrincipalPoint(1980.0, 1200.0);

    Camera camera1(CameraIntrinsicsType::DivisionUndistortion);
    camera1.setFocalLength(kFocalLength);
    camera1.setPrincipalPoint(kPrincipalPoint.x(), kPrincipalPoint.y());
    camera1.rIntrinsics()[DivisionUndistortionCameraModel::K] = kUndistortion;

    Camera camera2 = camera1;
    camera2.setOrientationFromAngleAxis(Vector3d(-0.1, -0.4, 0.3));
    camera2.setPosition(Vector3d(0.8, 0.2, 0.1));

    const Vector4d point(-2.3, 1.7, 6., 1.);
    Vector2d feature1, feature2;
    const double depth1 = camera1.projectPoint(point, feature1);
    const double depth2 = camera2.projectPoint(point, feature2);
    CHECK_GT(depth1, 0.0);
    CHECK_GT(depth2, 0.0);
    // Maybe check features too?

    const Vector3d gt_ray1 =
        (point.hnormalized() - camera1.position()).normalized();
    const Vector3d gt_ray2 =
        (point.hnormalized() - camera2.position()).normalized();
    const Vector3d ray1 = camera1.pixelToUnitDepthRay(feature1).normalized();
    const Vector3d ray2 = camera2.pixelToUnitDepthRay(feature2).normalized();

    const double angle1 = math::radToDeg(std::acos(gt_ray1.dot(ray1)));
    const double angle2 = math::radToDeg(std::acos(gt_ray2.dot(ray2)));
    CHECK_LT(std::abs(angle1), 1e-6);
    CHECK_LT(std::abs(angle2), 1e-6);
}

TEST(DivisionUndistortionCameraModel, NoDistortion)
{
    constexpr double kFocalLength = 3587.6;
    const Vector2d kPrincipalPoint(1980.0, 1200.0);

    Camera camera1{CameraIntrinsicsType::DivisionUndistortion};
    camera1.setFocalLength(kFocalLength);
    camera1.setPrincipalPoint(kPrincipalPoint.x(), kPrincipalPoint.y());
    camera1.setOrientationFromAngleAxis(Vector3d(-0.1, -0.4, 0.3));
    camera1.setPosition(Vector3d(0.8, 0.2, 0.1));

    Camera camera2{CameraIntrinsicsType::Pinhole};
    camera2.setPrincipalPoint(kPrincipalPoint.x(), kPrincipalPoint.y());
    camera2.setFocalLength(kFocalLength);
    camera2.setOrientationFromAngleAxis(Vector3d(-0.1, -0.4, 0.3));
    camera2.setPosition(Vector3d(0.8, 0.2, 0.1));

    const Vector4d point(-2.3, 1.7, 6, 1.0);
    Vector2d feature1, feature2;
    const double depth1 = camera1.projectPoint(point, feature1);
    const double depth2 = camera2.projectPoint(point, feature2);
    EXPECT_DOUBLE_EQ(depth1, depth2);
    EXPECT_DOUBLE_EQ(feature1.x(), feature2.x());
    EXPECT_DOUBLE_EQ(feature1.y(), feature2.y());

    Vector2d undistorted_pixel;
    DivisionUndistortionCameraModel::DistortedPixelToUndistortedPixel(
        camera1.intrinsics(), feature1.data(), undistorted_pixel.data());
    EXPECT_DOUBLE_EQ(feature1.x(), undistorted_pixel.x());
    EXPECT_DOUBLE_EQ(feature1.y(), undistorted_pixel.y());
}
