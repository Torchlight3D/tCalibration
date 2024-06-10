#include <Eigen/Core>
#include <gtest/gtest.h>

#include <tCamera/DoubleSphereCameraModel>

using namespace tl;

using Eigen::Vector2d;
using Eigen::Vector3d;

TEST(DoubleSphereCameraModel, InternalParameterGettersAndSetters)
{
    DoubleSphereCameraModel camera;

    // Check type
    EXPECT_EQ(camera.type(), CameraIntrinsicsType::DoubleSphere);

    // Check that default values are set
    EXPECT_EQ(camera.focalLength(), 1.0);
    EXPECT_EQ(camera.aspectRatio(), 1.0);
    EXPECT_EQ(camera.skew(), 0.0);
    EXPECT_EQ(camera.principalPointX(), 0.0);
    EXPECT_EQ(camera.principalPointY(), 0.0);
    EXPECT_EQ(camera.alpha(), 0.5);
    EXPECT_EQ(camera.xi(), 0.);

    // Set parameters to different values, and check that they were updated.
    constexpr auto kFocalLength{0.5 * 805.};
    constexpr auto kAspectRatio{805. / 800.};
    constexpr auto kSkew{0.};
    constexpr auto kPrincipalPointX{505.};
    constexpr auto kPrincipalPointY{509.};
    constexpr auto kAlpha{0.5 * -0.150694};
    constexpr auto kXi{0.5 * 1.48785};

    camera.setFocalLength(kFocalLength);
    camera.setAspectRatio(kAspectRatio);
    camera.setSkew(kSkew);
    camera.setPrincipalPoint(kPrincipalPointX, kPrincipalPointY);
    camera.setDistortion(kAlpha, kXi);

    EXPECT_EQ(camera.focalLength(), kFocalLength);
    EXPECT_EQ(camera.aspectRatio(), kAspectRatio);
    EXPECT_EQ(camera.skew(), kSkew);
    EXPECT_EQ(camera.principalPointX(), kPrincipalPointX);
    EXPECT_EQ(camera.principalPointY(), kPrincipalPointY);
    EXPECT_EQ(camera.alpha(), kAlpha);
    EXPECT_EQ(camera.xi(), kXi);
}

TEST(DoubleSphereCameraModel, SetFromCameraMetaDatas)
{
    // Test to ensure that the camera intrinsics are being set appropriately.
    auto TestCameraSetFromMeta = [](const CameraMetaData& meta) {
        const DoubleSphereCameraModel default_camera;

        DoubleSphereCameraModel camera;
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

        if (meta.skew.is_set) {
            EXPECT_EQ(camera.skew(), meta.skew.value[0]);
        }
        else {
            EXPECT_EQ(camera.skew(), default_camera.skew());
        }

        if (meta.radial_distortion.is_set) {
            EXPECT_EQ(camera.alpha(), meta.radial_distortion.value[0]);
            EXPECT_EQ(camera.xi(), meta.radial_distortion.value[1]);
        }
        else {
            EXPECT_EQ(camera.xi(), default_camera.xi());
            EXPECT_EQ(camera.alpha(), default_camera.alpha());
        }
    };

    CameraMetaData meta;
    meta.focal_length.value[0] = 1000.0;
    meta.principal_point.value[0] = 400.0;
    meta.principal_point.value[1] = 300.0;
    meta.aspect_ratio.value[0] = 1.01;
    meta.skew.value[0] = 0.01;
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

TEST(DoubleSphereCameraModel, GetSubsetFromOptimizeIntrinsicsType)
{
    using _Type = OptimizeIntrinsicsType;

    DoubleSphereCameraModel camera;
    std::vector<int> indices;

    indices = camera.fixedParameterIndices(_Type::None);
    EXPECT_EQ(indices.size(), camera.numParameters());

    // Focal length
    indices = camera.fixedParameterIndices(_Type::FocalLength);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, DoubleSphereCameraModel::Fx);
    }

    // Principal point
    indices = camera.fixedParameterIndices(_Type::PrincipalPoint);
    EXPECT_EQ(indices.size(), camera.numParameters() - 2);
    for (const auto& index : indices) {
        EXPECT_NE(index, DoubleSphereCameraModel::Cx);
        EXPECT_NE(index, DoubleSphereCameraModel::Cy);
    }

    // Aspect ratio
    indices = camera.fixedParameterIndices(_Type::AspectRatio);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, DoubleSphereCameraModel::YX);
    }

    // Skew
    indices = camera.fixedParameterIndices(_Type::Skew);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, DoubleSphereCameraModel::Skew);
    }

    // Radial distortion
    indices = camera.fixedParameterIndices(_Type::RadialDistortion);
    EXPECT_EQ(indices.size(), camera.numParameters() - 2);
    for (const auto& index : indices) {
        EXPECT_NE(index, DoubleSphereCameraModel::Alpha);
        EXPECT_NE(index, DoubleSphereCameraModel::Xi);
    }
}

class DoubleSphereCameraFixture : public ::testing::Test
{
protected:
    void SetUp() override
    {
        constexpr double kFocalLength = 364.4687315519798;
        constexpr double kPrincipalPoint[2] = {620.8633446632857,
                                               512.7479213012696};

        _camera.setFocalLength(kFocalLength);
        _camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    }

    // Test the projection functions of the camera model by testing pixels on a
    // grid in the entire image to ensure that the effects of lens distortion at
    // the edges of the image are modeled correctly.
    void TestReprojection() const
    {
        constexpr double kTolerance = 1e-6;
        const double kNormalizedTolerance = kTolerance / _camera.focalLength();
        constexpr int kImageWidth = 1280;
        constexpr int kImageHeight = 1024;
        constexpr double kMinDepth = 2.0;
        constexpr double kMaxDepth = 25.0;

        // Ensure the image -> camera -> image transformation works.
        for (double x = 0.0; x < kImageWidth; x += 10.0) {
            for (double y = 0.0; y < kImageHeight; y += 10.0) {
                const Vector2d pixel{x, y};
                const Vector3d normalized_ray = _camera.imageToSpace(pixel);

                // Test the reprojection at several depths.
                for (double depth = kMinDepth; depth < kMaxDepth; depth += 1.) {
                    const Vector3d point = normalized_ray * depth;
                    const Vector2d reprojected_pixel =
                        _camera.spaceToImage(point);

                    // Expect the reprojection to be close.
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
                    Vector3d normalized_ray = _camera.imageToSpace(pixel);
                    normalized_ray /= normalized_ray[2];
                    const Vector3d reprojected_point = normalized_ray * depth;

                    // Expect the reprojection to be close.
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
    DoubleSphereCameraModel _camera;
};

TEST_F(DoubleSphereCameraFixture, ReprojectionDistortion)
{
    _camera.setDistortion(0.572501301952097, -0.2725048615813684);
    TestReprojection();
}
