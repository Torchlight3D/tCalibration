#include <Eigen/Core>
#include <gtest/gtest.h>

#include <tCamera/OrthographicCameraModel>

using namespace tl;

using Eigen::Vector2d;
using Eigen::Vector3d;

TEST(OrthographicCameraModel, InternalParameterGettersAndSetters)
{
    OrthographicCameraModel camera;
    // Check type
    EXPECT_EQ(camera.type(), CameraIntrinsicsType::Orthographic);

    // Check that default values are set
    EXPECT_EQ(camera.focalLength(), 1.);
    EXPECT_EQ(camera.aspectRatio(), 1.);
    EXPECT_EQ(camera.skew(), 0.);
    EXPECT_EQ(camera.principalPointX(), 0.);
    EXPECT_EQ(camera.principalPointY(), 0.);
    EXPECT_EQ(camera.radialDistortion1(), 0.);
    EXPECT_EQ(camera.radialDistortion2(), 0.);

    constexpr double kFocalLength{3000.};
    constexpr double kAspectRatio{1.1};
    constexpr double kSkew{0.1};
    constexpr double kPrincipalPointX{300.};
    constexpr double kPrincipalPointY{400.};
    constexpr double kK1{1e-2};
    constexpr double kK2{1e-3};

    // Set parameters to different values.
    camera.setFocalLength(kFocalLength);
    camera.setAspectRatio(kAspectRatio);
    camera.setSkew(kSkew);
    camera.setPrincipalPoint(kPrincipalPointX, kPrincipalPointY);
    camera.setRadialDistortion(kK1, kK2);

    // Check that the values were updated.
    EXPECT_EQ(camera.focalLength(), kFocalLength);
    EXPECT_EQ(camera.aspectRatio(), kAspectRatio);
    EXPECT_EQ(camera.skew(), kSkew);
    EXPECT_EQ(camera.principalPointX(), kPrincipalPointX);
    EXPECT_EQ(camera.principalPointY(), kPrincipalPointY);
    EXPECT_EQ(camera.radialDistortion1(), kK1);
    EXPECT_EQ(camera.radialDistortion2(), kK2);
}

// Gradually add one prior at a time and ensure that the method still works. We
// test before and after setting the "is_set" member variable to true to ensure
// that setting the value of priors when is_set=false is handled properly.
TEST(OrthographicCameraModel, SetFromCameraMetaData)
{
    // Test to ensure that the camera intrinsics are being set appropriately.
    auto TestCameraSetFromMeta = [](const CameraMetaData& meta) {
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
            EXPECT_EQ(camera.cx(), meta.principal_point.value[0]);
            EXPECT_EQ(camera.cy(), meta.principal_point.value[1]);
        }
        else {
            EXPECT_EQ(camera.cx(), default_camera.cx());
            EXPECT_EQ(camera.cy(), default_camera.cy());
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
            EXPECT_EQ(camera.k1(), meta.radial_distortion.value[0]);
            EXPECT_EQ(camera.k2(), meta.radial_distortion.value[1]);
        }
        else {
            EXPECT_EQ(camera.k1(), default_camera.k1());
            EXPECT_EQ(camera.k2(), default_camera.k2());
        }
    };

    CameraMetaData meta;
    meta.focal_length.value[0] = 30000;
    meta.principal_point.value[0] = 400.0;
    meta.principal_point.value[1] = 300.0;
    meta.skew.value[0] = 0.0;
    meta.aspect_ratio.value[0] = 1.001;
    meta.radial_distortion.value[0] = 0.01;
    meta.radial_distortion.value[1] = 0.001;

    TestCameraSetFromMeta(meta);

    meta.focal_length.is_set = true;
    TestCameraSetFromMeta(meta);

    meta.principal_point.is_set = true;
    TestCameraSetFromMeta(meta);

    meta.aspect_ratio.is_set = true;
    TestCameraSetFromMeta(meta);

    meta.skew.is_set = true;
    TestCameraSetFromMeta(meta);

    meta.radial_distortion.is_set = true;
    TestCameraSetFromMeta(meta);
}

TEST(OrthographicCameraModel, GetSubsetFromOptimizeIntrinsicsType)
{
    using _Type = OptimizeIntrinsicsType;

    OrthographicCameraModel camera;
    std::vector<int> indices;

    indices = camera.fixedParameterIndices(_Type::None);
    EXPECT_EQ(indices.size(), camera.numParameters());

    // Focal length
    indices = camera.fixedParameterIndices(_Type::FocalLength);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, OrthographicCameraModel::Fx);
    }

    // Principal point
    indices = camera.fixedParameterIndices(_Type::PrincipalPoint);
    EXPECT_EQ(indices.size(), camera.numParameters() - 2);
    for (const auto& index : indices) {
        EXPECT_NE(index, OrthographicCameraModel::Cx);
        EXPECT_NE(index, OrthographicCameraModel::Cy);
    }

    // Aspect ratio
    indices = camera.fixedParameterIndices(_Type::AspectRatio);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, OrthographicCameraModel::YX);
    }

    // Skew
    indices = camera.fixedParameterIndices(_Type::Skew);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, OrthographicCameraModel::Skew);
    }

    // Radial distortion
    indices = camera.fixedParameterIndices(_Type::RadialDistortion);
    EXPECT_EQ(indices.size(), camera.numParameters() - 2);
    for (const auto& index : indices) {
        EXPECT_NE(index, OrthographicCameraModel::K1);
        EXPECT_NE(index, OrthographicCameraModel::K2);
    }

    // Tangential distortion
    indices = camera.fixedParameterIndices(_Type::TangentialDistortion);
    EXPECT_EQ(indices.size(), camera.numParameters());
}

class OrthographicCameraFixture : public ::testing::Test
{
protected:
    void SetUp() override
    {
        constexpr double kPrincipalPoint[2] = {1180.0, 1010.0};
        // This is actually magnification / pixel_pitch, e.g. 0.08/2e-6
        constexpr double kFocalLength = 40000.0;

        _camera.setFocalLength(kFocalLength);
        _camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    }

    // Test the projection functions of the camera model by testing pixels on a
    // grid in the entire image to ensure that the effects of lens distortion at
    // the edges of the image are modeled correctly.
    void TestReprojection() const
    {
        constexpr double kTolerance = 1e-5;
        constexpr double kNormalizedTolerance = kTolerance;
        constexpr int kImageWidth = 2560;
        constexpr int kImageHeight = 1920;

        // Ensure the camera -> image -> camera transformation works.
        for (double x = -0.01; x < 0.01; x += 0.001) {
            for (double y = -0.01; y < 0.01; y += 0.001) {
                const Vector3d point{x, y, 1.};
                const Vector2d pixel = _camera.spaceToImage(point);
                const Vector3d ray = _camera.imageToSpace(pixel);

                EXPECT_LT((point - ray).norm(), kNormalizedTolerance)
                    << "GT pixel: " << point.transpose()
                    << "\n"
                       "Reprojected pixel: "
                    << ray.transpose();
            }
        }

        // Ensure the image -> camera -> image transformation works.
        for (double x = 0.0; x < kImageWidth; x += 20.0) {
            for (double y = 0.0; y < kImageHeight; y += 20.0) {
                const Vector2d pixel{x, y};
                const Vector3d ray = _camera.imageToSpace(pixel);
                const Vector2d reprojected_pixel = _camera.spaceToImage(ray);

                EXPECT_LT((pixel - reprojected_pixel).norm(), kTolerance)
                    << "GT pixel: " << pixel.transpose()
                    << "\n"
                       "Reprojected pixel: "
                    << reprojected_pixel.transpose();
            }
        }
    }

protected:
    OrthographicCameraModel _camera;
};

TEST_F(OrthographicCameraFixture, ReprojectionNoDistortion)
{
    _camera.setRadialDistortion(0., 0.);
    TestReprojection();
}

TEST_F(OrthographicCameraFixture, ReprojectionOneDistortion)
{
    _camera.setRadialDistortion(1e-4, 0.);
    TestReprojection();
}

TEST_F(OrthographicCameraFixture, ReprojectionTwoDistortion)
{
    _camera.setRadialDistortion(1e-4, 1e-5);
    TestReprojection();
}
