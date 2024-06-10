#include <Eigen/Core>
#include <gtest/gtest.h>

#include <tCamera/FovCameraModel>

using namespace tl;

using Eigen::Vector2d;
using Eigen::Vector3d;

TEST(FOVCameraModel, InternalParameterGettersAndSetters)
{
    FovCameraModel camera;
    // Check type
    EXPECT_EQ(camera.type(), CameraIntrinsicsType::Fov);

    // Check that default values are set
    EXPECT_EQ(camera.focalLength(), 1.);
    EXPECT_EQ(camera.aspectRatio(), 1.);
    EXPECT_EQ(camera.principalPointX(), 0.);
    EXPECT_EQ(camera.principalPointY(), 0.);
    EXPECT_EQ(camera.omega(), FovCameraModel::kDefaultOmega);

    constexpr double kFocalLength{600.};
    constexpr double kAspectRatio{0.9};
    constexpr double kPrincipalPointX{300.};
    constexpr double kPrincipalPointY{400.};
    constexpr double kOmega{0.01};

    // Set parameters to different values.
    camera.setFocalLength(kFocalLength);
    camera.setAspectRatio(kAspectRatio);
    camera.setPrincipalPoint(kPrincipalPointX, kPrincipalPointY);
    camera.setRadialDistortion(kOmega);

    // Check that the values were updated.
    EXPECT_EQ(camera.focalLength(), kFocalLength);
    EXPECT_EQ(camera.aspectRatio(), kAspectRatio);
    EXPECT_EQ(camera.cx(), kPrincipalPointX);
    EXPECT_EQ(camera.cy(), kPrincipalPointY);
    EXPECT_EQ(camera.omega(), kOmega);
}

// Gradually add one meta at a time and ensure that the method still works. We
// test before and after setting the "is_set" member variable to true to ensure
// that setting the value of priors when is_set=false is handled properly.
TEST(FOVCameraModel, SetFromCameraMetaData)
{
    // Test to ensure that the camera intrinsics are being set appropriately.
    auto TestCameraSetFromMeta = [](const CameraMetaData& meta) {
        const FovCameraModel default_camera;
        FovCameraModel camera;
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
            EXPECT_EQ(camera.cx(), default_camera.cx());
            EXPECT_EQ(camera.cy(), default_camera.cy());
        }

        if (meta.aspect_ratio.is_set) {
            EXPECT_EQ(camera.aspectRatio(), meta.aspect_ratio.value[0]);
        }
        else {
            EXPECT_EQ(camera.aspectRatio(), default_camera.aspectRatio());
        }

        if (meta.radial_distortion.is_set) {
            EXPECT_EQ(camera.omega(), meta.radial_distortion.value[0]);
        }
        else {
            EXPECT_EQ(camera.omega(), default_camera.omega());
        }
    };

    CameraMetaData meta;
    meta.focal_length.value[0] = 1000.0;
    meta.principal_point.value[0] = 400.0;
    meta.principal_point.value[1] = 300.0;
    meta.aspect_ratio.value[0] = 1.01;
    meta.radial_distortion.value[0] = 0.01;

    TestCameraSetFromMeta(meta);

    meta.focal_length.is_set = true;
    TestCameraSetFromMeta(meta);

    meta.principal_point.is_set = true;
    TestCameraSetFromMeta(meta);

    meta.aspect_ratio.is_set = true;
    TestCameraSetFromMeta(meta);

    meta.radial_distortion.is_set = true;
    TestCameraSetFromMeta(meta);
}

TEST(FOVCameraModel, fixedParameterIndices)
{
    using _Type = OptimizeIntrinsicsType;

    FovCameraModel camera;
    std::vector<int> indices;

    indices = camera.fixedParameterIndices(_Type::None);
    EXPECT_EQ(indices.size(), camera.numParameters());

    // Focal length
    indices = camera.fixedParameterIndices(_Type::FocalLength);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, FovCameraModel::Fx);
    }

    // Principal point
    indices = camera.fixedParameterIndices(_Type::PrincipalPoint);
    EXPECT_EQ(indices.size(), camera.numParameters() - 2);
    for (const auto& index : indices) {
        EXPECT_NE(index, FovCameraModel::Cx);
        EXPECT_NE(index, FovCameraModel::Cy);
    }

    // Aspect ratio
    indices = camera.fixedParameterIndices(_Type::AspectRatio);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, FovCameraModel::YX);
    }

    // Skew
    indices = camera.fixedParameterIndices(_Type::RadialDistortion);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, FovCameraModel::Omega);
    }

    // Skew and tangential distortion
    indices = camera.fixedParameterIndices(_Type::Skew);
    EXPECT_EQ(indices.size(), camera.numParameters());
    indices = camera.fixedParameterIndices(_Type::TangentialDistortion);
    EXPECT_EQ(indices.size(), camera.numParameters());
}

class FovCameraFixture : public ::testing::Test
{
protected:
    void SetUp() override
    {
        constexpr double kPrincipalPoint[2]{600., 400.};
        constexpr double kFocalLength{1200.};

        _camera.setFocalLength(kFocalLength);
        _camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    }

    void TestProjection() const
    {
        constexpr double kTolerance{1e-5};
        const double kNormalizedTolerance = kTolerance / _camera.focalLength();
        constexpr int kImageWidth{1200};
        constexpr int kImageHeight{980};
        constexpr double kMinDepth{2.0};
        constexpr double kMaxDepth{25.0};

        // Ensure the image -> camera -> image transformation works.
        for (double x = 0.0; x < kImageWidth; x += 10.0) {
            for (double y = 0.0; y < kImageHeight; y += 10.0) {
                const Vector2d pixel{x, y};
                const Vector3d ray = _camera.imageToSpace(pixel);

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
    FovCameraModel _camera;
};

TEST_F(FovCameraFixture, ReprojectionNoDistortion)
{
    _camera.setRadialDistortion(0.);
    TestProjection();
}

TEST_F(FovCameraFixture, ReprojectionOneDistortionSmall)
{
    _camera.setRadialDistortion(1e-4);
    TestProjection();
}

TEST_F(FovCameraFixture, ReprojectionOneDistortionMedium)
{
    _camera.setRadialDistortion(1e-3);
    TestProjection();
}

TEST_F(FovCameraFixture, ReprojectionOneDistortionLarge)
{
    _camera.setRadialDistortion(1e-1);
    TestProjection();
}
