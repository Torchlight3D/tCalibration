#include <Eigen/Core>
#include <gtest/gtest.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <tCamera/coc/CataCamera>
#include <tCamera/OmnidirectionalCameraModel>

using namespace tl;

using Eigen::Vector2d;
using Eigen::Vector3d;

class OmnidirectionalCameraFixture : public ::testing::Test
{
protected:
    void SetUp() override
    {
        _camera.setFocalLength(758.355);
        _camera.setAspectRatio(0.999024);
        _camera.setPrincipalPoint(646.72, 395.001);
        _camera.setMirrorDistortion(0.894975);
        _camera.setTangentialDistortion(-0.00403995, 0.00610364);
        _camera.setRadialDistortion(-0.344504, 0.0984552);
    }

protected:
    OmnidirectionalCameraModel _camera;
};

TEST_F(OmnidirectionalCameraFixture, SpaceToImage)
{
    const Vector3d point3{0., 0., 1.};
    const Vector2d pixel = _camera.spaceToImage(point3);

    EXPECT_NEAR(_camera.principalPointX(), pixel(0), 1e-10);
    EXPECT_NEAR(_camera.principalPointY(), pixel(1), 1e-10);
}

TEST_F(OmnidirectionalCameraFixture, ImageToRay)
{
    const Vector2d px{_camera.principalPointX(), _camera.principalPointY()};
    const Vector3d ray = _camera.imageToSpace(px).normalized();

    EXPECT_NEAR(0., ray(0), 1e-10);
    EXPECT_NEAR(0., ray(1), 1e-10);
    EXPECT_NEAR(1., ray(2), 1e-10);
}

TEST_F(OmnidirectionalCameraFixture, Reprojection)
{
    constexpr int kSampleCount = 100;
    for (int i{0}; i < kSampleCount; ++i) {
        Vector3d point3 = Vector3d::Random();
        point3(2) = std::abs(point3(2));

        const auto pixel = _camera.spaceToImage(point3);
        auto reprojected = _camera.imageToSpace(pixel).normalized();

        point3.normalize();

        EXPECT_NEAR(point3(0), reprojected(0), 1e-2);
        EXPECT_NEAR(point3(1), reprojected(1), 1e-2);
        EXPECT_NEAR(point3(2), reprojected(2), 1e-2);
    }
}

TEST_F(OmnidirectionalCameraFixture, CamOdoCalConsistency)
{
    const camodocal::CataCamera camera_coc{
        "camera",    640,        480,     0.894975, -0.344504, 0.0984552,
        -0.00403995, 0.00610364, 758.355, 757.615,  646.72,    395.001};

    // TODO: Use random points in frustum
    constexpr int kSampleCount = 100;
    for (int i{0}; i < kSampleCount; ++i) {
        Vector3d point3 = Vector3d::Random();
        point3(2) = std::abs(point3(2));

        // Ensure both "world -> image -> world" map to a same result
        // CalibKit
        const auto pixel = _camera.spaceToImage(point3);
        auto reprojected = _camera.imageToSpace(pixel).normalized();

        // CamOdoCal
        Vector2d pixel_coc;
        camera_coc.spaceToPlane(point3, pixel_coc);

        Vector3d reprojected_coc;
        camera_coc.liftProjective(pixel_coc, reprojected_coc);
        reprojected_coc.normalize();

        point3.normalize();

        EXPECT_NEAR(point3(0), reprojected(0), 1e-2);
        EXPECT_NEAR(point3(1), reprojected(1), 1e-2);
        EXPECT_NEAR(point3(2), reprojected(2), 1e-2);
    }
}

TEST(OmnidirectionalCameraModel, KalibrCompatible)
{
    // TODO: Finish here
    GTEST_SKIP();

    OmnidirectionalCameraModel camera_kit;
    // Reference: before any edit
    //    camera_kit.setFocalLength(1312.82);
    //    camera_kit.setAspectRatio(0.998284);
    //    camera_kit.setPrincipalPoint(332.94, 190.013);
    //    camera_kit.setMirrorDistortion(3.09005);
    //    camera_kit.setRadialDistortion(0.945562, 7.8695);
    //    camera_kit.setTangentialDistortion(0.00455797, -0.00202254);

    camera_kit.setFocalLength(998.23);
    camera_kit.setAspectRatio(0.998262);
    camera_kit.setPrincipalPoint(332.936, 190.257);
    camera_kit.setMirrorDistortion(2.11142);
    camera_kit.setRadialDistortion(0.202064, 1.1038);
    camera_kit.setTangentialDistortion(0.00334523, -0.00141154);

    OmnidirectionalCameraModel camera_cv;
    camera_cv.setFocalLength(653.49);
    camera_cv.setAspectRatio(0.998528);
    camera_cv.setPrincipalPoint(331.922, 206.101);
    camera_cv.setMirrorDistortion(1.03232);
    camera_cv.setRadialDistortion(-0.249668, 0.113888);
    camera_cv.setTangentialDistortion(0.00239189, -0.00198596);

    OmnidirectionalCameraModel camera_kalibr;
    camera_kalibr.setFocalLength(837.4664892993342);
    camera_kalibr.setAspectRatio(0.997876);
    camera_kalibr.setPrincipalPoint(332.00617987478984, 191.08042516943507);
    camera_kalibr.setMirrorDistortion(1.60826862964);
    camera_kalibr.setRadialDistortion(-0.05752771998045863,
                                      0.22237550975545115);
    camera_kalibr.setTangentialDistortion(0.0018744794789161787,
                                          1.2160264129479937e-05);

    cv::Mat detection = cv::Mat::zeros(480, 640, CV_8UC3);

    constexpr int kSampleCount = 1000;
    for (int i{0}; i < kSampleCount; ++i) {
        Vector3d point3 = Vector3d::Random();
        point3(2) = std::abs(point3(2));

        // CalibKit
        const auto pixel_kit = camera_kit.spaceToImage(point3);
        auto reprojected_kit = camera_kit.imageToSpace(pixel_kit);

        // OpenCV
        const auto pixel_cv = camera_cv.spaceToImage(point3);
        auto reprojected_cv = camera_cv.imageToSpace(pixel_cv);

        // Kalibr
        const auto pixel_kalibr = camera_kalibr.spaceToImage(point3);
        auto reprojected_kalibr = camera_kalibr.imageToSpace(pixel_kalibr);

        LOG(INFO) << "Project by CalibKit intrisic: " << pixel_kit.transpose()
                  << "; "
                     //                     "Project by OpenCV intrisic: "
                     //                  << pixel_cv.transpose()
                     //                  << "; "
                     "Project by Kalibr intrisic: "
                  << pixel_kalibr.transpose();

        if ((pixel_kit.array() > 0).all() && (pixel_kalibr.array() > 0).all()) {
            cv::drawMarker(detection, cv::Point(pixel_kit.x(), pixel_kit.y()),
                           CV_RGB(255, 0, 0), cv::MARKER_CROSS, 4);
            cv::drawMarker(detection,
                           cv::Point(pixel_kalibr.x(), pixel_kalibr.y()),
                           CV_RGB(0, 255, 0), cv::MARKER_CROSS, 4);
        }
    }

    cv::imwrite("detection.png", detection);

    EXPECT_TRUE(true);
}

TEST(OmnidirectionalCameraModel, OpenCVCompatible)
{
    // TODO: Finish here
    GTEST_SKIP();
}
