#include <Eigen/Core>
#include <gtest/gtest.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <tCamera/coc/EquidistantCamera>
#include <tCamera/FisheyeCameraModel>

using namespace tl;

using Eigen::Vector2d;
using Eigen::Vector3d;

TEST(FisheyeCameraModel, CreateFromName)
{
    using theFactory = ::factory::Registry<CameraIntrinsics>;

    ASSERT_TRUE(theFactory::CanNew(FisheyeCameraModel::kName));
    ASSERT_TRUE(theFactory::New(FisheyeCameraModel::kName));
}

TEST(FisheyeCameraModel, ParametersSetterAndGetter)
{
    FisheyeCameraModel camera;
    // Check type
    EXPECT_EQ(camera.type(), CameraIntrinsicsType::Fisheye);

    // Check all default values are set
    EXPECT_EQ(camera.focalLength(), 1.);
    EXPECT_EQ(camera.aspectRatio(), 1.);
    EXPECT_EQ(camera.skew(), 0.);
    EXPECT_EQ(camera.cx(), 0.);
    EXPECT_EQ(camera.cy(), 0.);
    EXPECT_EQ(camera.k1(), 0.);
    EXPECT_EQ(camera.k2(), 0.);
    EXPECT_EQ(camera.k3(), 0.);
    EXPECT_EQ(camera.k4(), 0.);

    constexpr double kFocalLength{600.};
    constexpr double kAspectRatio{0.9};
    constexpr double kSkew{0.01};
    constexpr double kPrincipalPointX{300.};
    constexpr double kPrincipalPointY{400.};
    constexpr double kK1{0.01};
    constexpr double kK2{0.001};
    constexpr double kK3{0.002};
    constexpr double kK4{0.003};

    // Set parameters to different values, and check that they were updated.
    camera.setFocalLength(kFocalLength);
    camera.setAspectRatio(kAspectRatio);
    camera.setSkew(kSkew);
    camera.setPrincipalPoint(kPrincipalPointX, kPrincipalPointY);
    camera.setRadialDistortion(kK1, kK2, kK3, kK4);

    EXPECT_EQ(camera.focalLength(), kFocalLength);
    EXPECT_EQ(camera.aspectRatio(), kAspectRatio);
    EXPECT_EQ(camera.skew(), kSkew);
    EXPECT_EQ(camera.cx(), kPrincipalPointX);
    EXPECT_EQ(camera.cy(), kPrincipalPointY);
    EXPECT_EQ(camera.k1(), kK1);
    EXPECT_EQ(camera.k2(), kK2);
    EXPECT_EQ(camera.k3(), kK3);
    EXPECT_EQ(camera.k4(), kK4);
}

TEST(FisheyeCameraModel, SetFromCameraMetaData)
{
    auto TestCameraSetFromMeta = [](const CameraMetaData& meta) {
        const FisheyeCameraModel default_camera;

        FisheyeCameraModel camera;
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

        if (meta.skew.has_value()) {
            EXPECT_EQ(camera.skew(), meta.skew.value()[0]);
        }
        else {
            EXPECT_EQ(camera.skew(), default_camera.skew());
        }

        if (meta.radialDistortion.has_value()) {
            EXPECT_EQ(camera.k1(), meta.radialDistortion.value()[0]);
            EXPECT_EQ(camera.k2(), meta.radialDistortion.value()[1]);
            EXPECT_EQ(camera.k3(), meta.radialDistortion.value()[2]);
            EXPECT_EQ(camera.k4(), meta.radialDistortion.value()[3]);
        }
        else {
            EXPECT_EQ(camera.k1(), default_camera.k1());
            EXPECT_EQ(camera.k2(), default_camera.k2());
            EXPECT_EQ(camera.k3(), default_camera.k3());
            EXPECT_EQ(camera.k4(), default_camera.k4());
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

    meta.skew = {1e-2};
    TestCameraSetFromMeta(meta);

    meta.radialDistortion = {1e-2, 1e-3, 1.1e-3, 1.2e-3};
    TestCameraSetFromMeta(meta);
}

TEST(FisheyeCameraModel, ConstantParameterIndices)
{
    using _Type = OptimizeIntrinsicsType;

    FisheyeCameraModel camera;
    std::vector<int> indices;

    indices = camera.fixedParameterIndices(_Type::None);
    EXPECT_EQ(indices.size(), camera.numParameters());

    // Focal length
    indices = camera.fixedParameterIndices(_Type::FocalLength);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, FisheyeCameraModel::Fx);
    }

    // Aspect ratio
    indices = camera.fixedParameterIndices(_Type::AspectRatio);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, FisheyeCameraModel::YX);
    }

    // Principal points
    indices = camera.fixedParameterIndices(_Type::PrincipalPoint);
    EXPECT_EQ(indices.size(), camera.numParameters() - 2);
    for (const auto& index : indices) {
        EXPECT_NE(index, FisheyeCameraModel::Cx);
        EXPECT_NE(index, FisheyeCameraModel::Cy);
    }

    // Skew works
    indices = camera.fixedParameterIndices(_Type::Skew);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, FisheyeCameraModel::Skew);
    }

    // Radial distortion
    indices = camera.fixedParameterIndices(_Type::RadialDistortion);
    EXPECT_EQ(indices.size(), camera.numParameters() - 4);
    for (const auto& index : indices) {
        EXPECT_NE(index, FisheyeCameraModel::K1);
        EXPECT_NE(index, FisheyeCameraModel::K2);
        EXPECT_NE(index, FisheyeCameraModel::K3);
        EXPECT_NE(index, FisheyeCameraModel::K4);
    }

    // Tangential distortion
    indices = camera.fixedParameterIndices(_Type::TangentialDistortion);
    EXPECT_EQ(indices.size(), camera.numParameters());
}

class FisheyeCameraFixture : public ::testing::Test
{
protected:
    void SetUp() override
    {
        constexpr double kPrincipalPoint[2] = {600., 400.};
        constexpr double kFocalLength = 1200.;

        _camera.setFocalLength(kFocalLength);
        _camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    }

    void TestReprojection() const
    {
        constexpr double kTolerance = 1e-5;
        const double kNormalizedTolerance = kTolerance / _camera.focalLength();
        constexpr int kImageWidth = 1200;
        constexpr int kImageHeight = 980;
        constexpr double kMinDepth = 2.;
        constexpr double kMaxDepth = 25.;

        // Ensure the image -> camera -> image transformation works.
        for (double x = 0.0; x < kImageWidth; x += 10.0) {
            for (double y = 0.0; y < kImageHeight; y += 10.0) {
                const Vector2d pixel{x, y};
                const Vector3d ray = _camera.imageToSpace(pixel);

                // Test the reprojection at several depths.
                for (double depth = kMinDepth; depth < kMaxDepth; depth += 1.) {
                    const Vector3d point = ray * depth;
                    const Vector2d reprojected_pixel =
                        _camera.spaceToImage(point);

                    EXPECT_LT((pixel - reprojected_pixel).norm(), kTolerance)
                        << "GT pixel: " << pixel.transpose()
                        << "\n"
                           "Reprojected pixel: "
                        << reprojected_pixel.transpose();
                }
            }
        }

        // Ensure the camera -> image -> camera transformation works.
        for (double x = -0.8; x < 0.8; x += 0.1) {
            for (double y = -0.8; y < 0.8; y += 0.1) {
                for (double depth = kMinDepth; depth < kMaxDepth; depth += 1.) {
                    const Vector3d point{x, y, depth};
                    const Vector2d pixel = _camera.spaceToImage(point);
                    const Vector3d ray = _camera.imageToSpace(pixel);
                    const Vector3d reprojected_point = ray * depth;

                    EXPECT_LT((point - reprojected_point).norm(),
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
    FisheyeCameraModel _camera;
};

TEST_F(FisheyeCameraFixture, ReprojectionNoDistortion)
{
    _camera.setRadialDistortion(0., 0., 0., 0.);
    TestReprojection();
}

TEST_F(FisheyeCameraFixture, ReprojectionOneDistortion)
{
    _camera.setRadialDistortion(0.01, 0., 0., 0.);
    TestReprojection();
}

TEST_F(FisheyeCameraFixture, ReprojectionTwoDistortion)
{
    _camera.setRadialDistortion(0.01, 0.001, 0., 0.);
    TestReprojection();
}

TEST_F(FisheyeCameraFixture, ReprojectionThreeDistortion)
{
    _camera.setRadialDistortion(0.01, 0.001, 0.001, 0.);
    TestReprojection();
}

TEST_F(FisheyeCameraFixture, ReprojectionFourDistortion)
{
    _camera.setRadialDistortion(0.01, 0.001, 0.001, 0.001);
    TestReprojection();
}

class cocFisheyeCameraFixture : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // clang-format off
        _camera_coc = {"camera",
                        1280, 800,
                        -0.01648, -0.00203, 0.00069, -0.00048,
                        419.22826, 420.42160,
                        655.45487, 389.66377};
        // clang-format on
    }

protected:
    camodocal::EquidistantCamera _camera_coc;
};

TEST_F(cocFisheyeCameraFixture, CamOdoCalConsistency)
{
    FisheyeCameraModel camera;
    camera.setFocalLength(419.22826);
    camera.setAspectRatio(1.002847);
    camera.setPrincipalPoint(655.45487, 389.66377);
    camera.setRadialDistortion(-0.01648, -0.00203, 0.00069, -0.00048);

    constexpr int kSampleCount = 100;

    // FIXME: C style is not recommended
    std::srand((unsigned int)::time(nullptr));

    int count{0};
    for (int i{0}; i < kSampleCount; ++i) {
        Vector3d point3 = Vector3d::Random();
        point3(2) = std::abs(point3(2));

        // CalibKit
        const auto px = camera.spaceToImage(point3);
        auto ray_kit = camera.imageToSpace(px).normalized();

        // CamOdoCal
        Vector2d px_coc;
        _camera_coc.spaceToPlane(point3, px_coc);

        Vector3d ray_coc;
        _camera_coc.liftProjective(px_coc, ray_coc);
        ray_coc.normalize();

        if (!((ray_coc - ray_kit).cwiseAbs().array() < 1e-10).all()) {
            LOG(INFO) << "Original point: " << point3.transpose()
                      << "; "
                         "Reproject by CalibKit: "
                      << ray_kit.transpose()
                      << "; "
                         "Reproject by CamOdoCal: "
                      << ray_coc.transpose();

            ++count;
        }
    }

    LOG(INFO) << "There are " << count << " deviation points.";
    EXPECT_EQ(count, 0);
}

TEST_F(cocFisheyeCameraFixture, CamOdoCalReprojectConsistency)
{
    constexpr int kSampleCount = 100;

    int count{0};
    for (int i = 0; i < 100; ++i) {
        Eigen::Vector3d point = Eigen::Vector3d::Random();
        point(2) = std::abs(point(2));

        Eigen::Vector2d px;
        _camera_coc.spaceToPlane(point, px);

        Eigen::Vector3d ray;
        _camera_coc.liftProjective(px, ray);

        Vector3d ray_newton;
        _camera_coc.liftProjectiveByIteration1(px, ray_newton);

        Vector3d ray_iter;
        _camera_coc.liftProjectiveByIteration1(px, ray_iter);

        point.normalize();
        ray.normalize();
        ray_newton.normalize();
        ray_iter.normalize();

        if (!((point - ray_newton).cwiseAbs().array() < 1e-10).all()) {
            LOG(INFO) << "Origin: " << point.transpose()
                      << ", "
                         "Revise : "
                      << ray_newton.transpose();
            ++count;
        }
    }

    LOG(INFO) << "There are " << count << " deviation points.";
    EXPECT_EQ(count, 0);
}

TEST(FisheyeCameraModel, Playground)
{
    // Good-1-CalibKit
    // 0.0067743900959463966, 0.042858245977267943, -0.085987998031273952,
    // 0.063911157302371041

    // Good-1-Kalibr
    // 1.2352591910062178e-02, 1.8459255582963689e-02,
    // -2.1322294391136667e-02, 6.8752410586414838e-03

    // Bad-1-CalibKit
    // 0.019028151275260561, -0.04693368669301206, 0.16270783293226751,
    // -0.16537907477560224

    // Bad-1-Kalibr
    // 9.2997179392838806e-03, 3.0590423011149451e-02,
    // -3.2023628626212566e-02, 1.0416638741187919e-02

    FisheyeCameraModel camera_kit;
    camera_kit.setFocalLength(318.30227058009393);
    camera_kit.setAspectRatio(0.9999181798078266);
    camera_kit.setPrincipalPoint(324.963542409841, 200.2321543392214);
    camera_kit.setRadialDistortion(0.0109363, 0.00989819, 0.00153313,
                                   -0.00833513);

    FisheyeCameraModel camera_kalibr;
    camera_kalibr.setFocalLength(318.30227058009393);
    camera_kalibr.setAspectRatio(0.9999181798078266);
    camera_kalibr.setPrincipalPoint(324.963542409841, 200.2321543392214);
    camera_kalibr.setRadialDistortion(
        9.2997179392838806e-03, 3.0590423011149451e-02, -3.2023628626212566e-02,
        1.0416638741187919e-02);

    cv::Mat detection = cv::Mat::zeros(480, 640, CV_8UC3);

    constexpr int kSampleCount = 1000;
    for (int i{0}; i < kSampleCount; ++i) {
        Vector3d point3 = Vector3d::Random();
        point3(2) = std::abs(point3(2));

        const auto pixel_kit = camera_kit.spaceToImage(point3);

        const auto pixel_kalibr = camera_kalibr.spaceToImage(point3);

        if ((pixel_kit.array() > 0).all() && (pixel_kalibr.array() > 0).all()) {
            cv::drawMarker(detection, cv::Point(pixel_kit.x(), pixel_kit.y()),
                           CV_RGB(255, 0, 0), cv::MARKER_CROSS, 4);
            cv::drawMarker(detection,
                           cv::Point(pixel_kalibr.x(), pixel_kalibr.y()),
                           CV_RGB(0, 255, 0), cv::MARKER_CROSS, 4);
        }
    }

    cv::imwrite("space_to_pixel.png", detection);

    EXPECT_TRUE(true);
}
