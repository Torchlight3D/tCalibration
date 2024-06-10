#include <Eigen/Core>
#include <gtest/gtest.h>

#include <tCamera/PinholeCameraModel>

using namespace tl;

using Eigen::Vector2d;
using Eigen::Vector3d;

TEST(PinholeCameraModel, InternalParameterGettersAndSetters)
{
    PinholeCameraModel camera;
    // Check type
    EXPECT_EQ(camera.type(), CameraIntrinsicsType::Pinhole);

    // Check that default values are set
    EXPECT_EQ(camera.focalLength(), 1.0);
    EXPECT_EQ(camera.aspectRatio(), 1.0);
    EXPECT_EQ(camera.skew(), 0.0);
    EXPECT_EQ(camera.principalPointX(), 0.0);
    EXPECT_EQ(camera.principalPointY(), 0.0);
    EXPECT_EQ(camera.radialDistortion1(), 0.0);
    EXPECT_EQ(camera.radialDistortion2(), 0.0);

    constexpr double kFocalLength{600.};
    constexpr double kAspectRatio{0.9};
    constexpr double kSkew{0.01};
    constexpr double kPrincipalPointX{300.};
    constexpr double kPrincipalPointY{400.};
    constexpr double kK1{0.01};
    constexpr double kK2{0.001};

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

TEST(PinholeCameraModel, SetFromCameraMetaData)
{
    auto TestCameraSetFromMeta = [](const CameraMetaData& meta) {
        const PinholeCameraModel default_camera;

        PinholeCameraModel camera;
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
        }
        else {
            EXPECT_EQ(camera.k1(), default_camera.k1());
            EXPECT_EQ(camera.k2(), default_camera.k2());
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

    meta.radialDistortion = {1e-2, 1e-3, 0., 0.};
    TestCameraSetFromMeta(meta);
}

TEST(PinholeCameraModel, ConstantParameterIndices)
{
    using _Type = OptimizeIntrinsicsType;

    PinholeCameraModel camera;
    std::vector<int> indices;

    indices = camera.fixedParameterIndices(_Type::None);
    EXPECT_EQ(indices.size(), camera.numParameters());

    // Focal length
    indices = camera.fixedParameterIndices(_Type::FocalLength);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, PinholeCameraModel::Fx);
    }

    // Principal point
    indices = camera.fixedParameterIndices(_Type::PrincipalPoint);
    EXPECT_EQ(indices.size(), camera.numParameters() - 2);
    for (const auto& index : indices) {
        EXPECT_NE(index, PinholeCameraModel::Cx);
        EXPECT_NE(index, PinholeCameraModel::Cy);
    }

    // Aspect ratio
    indices = camera.fixedParameterIndices(_Type::AspectRatio);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, PinholeCameraModel::YX);
    }

    // Skew
    indices = camera.fixedParameterIndices(_Type::Skew);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, PinholeCameraModel::Skew);
    }

    // Radial distortion
    indices = camera.fixedParameterIndices(_Type::RadialDistortion);
    EXPECT_EQ(indices.size(), camera.numParameters() - 2);
    for (const auto& index : indices) {
        EXPECT_NE(index, PinholeCameraModel::K1);
        EXPECT_NE(index, PinholeCameraModel::K2);
    }

    // Tangential distortion
    indices = camera.fixedParameterIndices(_Type::TangentialDistortion);
    EXPECT_EQ(indices.size(), camera.numParameters());
}

class PinholeCameraFixture : public ::testing::Test
{
protected:
    void SetUp() override
    {
        constexpr double kPrincipalPoint[2] = {600.0, 400.0};
        constexpr double kFocalLength = 1200;

        _camera.setFocalLength(kFocalLength);
        _camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    }

    // Test the projection functions of the camera model by testing pixels on a
    // grid in the entire image to ensure that the effects of lens distortion at
    // the edges of the image are modeled correctly.
    void TestReprojection() const
    {
        constexpr double kTolerance = 1e-5;
        const double kNormalizedTolerance = kTolerance / _camera.focalLength();
        constexpr int kImageWidth = 1200;
        constexpr int kImageHeight = 980;
        constexpr double kMinDepth = 2.0;
        constexpr double kMaxDepth = 25.0;

        // Ensure the image -> camera -> image transformation works.
        for (double x = 0.0; x < kImageWidth; x += 10.0) {
            for (double y = 0.0; y < kImageHeight; y += 10.0) {
                const Vector2d pixel{x, y};
                const Vector3d normalized_ray = _camera.imageToSpace(pixel);

                // Test the reprojection at several depths.
                for (double depth = kMinDepth; depth < kMaxDepth;
                     depth += 1.0) {
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
                    const Vector3d normalized_ray = _camera.imageToSpace(pixel);
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
    PinholeCameraModel _camera;
};

TEST_F(PinholeCameraFixture, ReprojectionNoDistortion)
{
    _camera.setRadialDistortion(0., 0.);
    TestReprojection();
}

TEST_F(PinholeCameraFixture, ReprojectionOneDistortion)
{
    _camera.setRadialDistortion(0.01, 0.);
    TestReprojection();
}

TEST_F(PinholeCameraFixture, ReprojectionTwoDistortion)
{
    _camera.setRadialDistortion(0.01, 0.001);
    TestReprojection();
}

#include <tCamera/PinholeRadialTangentialCameraModel>

TEST(PinholeRadialTangentialCameraModel, InternalParameterGettersAndSetters)
{
    PinholeRadialTangentialCameraModel camera;
    // Check type
    EXPECT_EQ(camera.type(), CameraIntrinsicsType::PinholeRadialTangential);

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

    constexpr double kFocalLength{600.};
    constexpr double kAspectRatio{0.9};
    constexpr double kSkew{0.01};
    constexpr double kPrincipalPointX{300.};
    constexpr double kPrincipalPointY{400.};
    constexpr double kK1{1e-2};
    constexpr double kK2{1e-3};
    constexpr double kK3{1e-4};
    constexpr double kT1{2e-2};
    constexpr double kT2{2e-3};

    // Set parameters to different values.
    camera.setFocalLength(kFocalLength);
    camera.setAspectRatio(kAspectRatio);
    camera.setSkew(kSkew);
    camera.setPrincipalPoint(kPrincipalPointX, kPrincipalPointY);
    camera.setRadialDistortion(kK1, kK2, kK3);
    camera.setTangentialDistortion(kT1, kT2);

    // Check that the values were updated.
    EXPECT_EQ(camera.focalLength(), kFocalLength);
    EXPECT_EQ(camera.aspectRatio(), kAspectRatio);
    EXPECT_EQ(camera.skew(), kSkew);
    EXPECT_EQ(camera.principalPointX(), kPrincipalPointX);
    EXPECT_EQ(camera.principalPointY(), kPrincipalPointY);
    EXPECT_EQ(camera.radialDistortion1(), kK1);
    EXPECT_EQ(camera.radialDistortion2(), kK2);
    EXPECT_EQ(camera.radialDistortion3(), kK3);
    EXPECT_EQ(camera.tangentialDistortion1(), kT1);
    EXPECT_EQ(camera.tangentialDistortion2(), kT2);
}

TEST(PinholeRadialTangentialCameraModel, SetFromCameraMetaData)
{
    auto TestCameraSetFromMeta = [](const CameraMetaData& meta) {
        const PinholeRadialTangentialCameraModel default_camera;

        PinholeRadialTangentialCameraModel camera;
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
        }
        else {
            EXPECT_EQ(camera.k1(), default_camera.k1());
            EXPECT_EQ(camera.k2(), default_camera.k2());
            EXPECT_EQ(camera.k3(), default_camera.k3());
        }

        if (meta.tangentialDistortion.has_value()) {
            EXPECT_EQ(camera.t1(), meta.tangentialDistortion.value()[0]);
            EXPECT_EQ(camera.t2(), meta.tangentialDistortion.value()[1]);
        }
        else {
            EXPECT_EQ(camera.t1(), default_camera.t1());
            EXPECT_EQ(camera.t2(), default_camera.t2());
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

    meta.radialDistortion = {1e-1, 1e-2, 1e-3, 0.};
    TestCameraSetFromMeta(meta);

    meta.tangentialDistortion = {1.1e-1, 1.1e-2};
    TestCameraSetFromMeta(meta);
}

TEST(PinholeRadialTangentialCameraModel, GetSubsetFromOptimizeIntrinsicsType)
{
    using _Type = OptimizeIntrinsicsType;

    PinholeRadialTangentialCameraModel camera;
    std::vector<int> indices;

    indices = camera.fixedParameterIndices(_Type::None);
    EXPECT_EQ(indices.size(), camera.numParameters());

    // Test that optimizing for focal length works correctly.
    indices = camera.fixedParameterIndices(_Type::FocalLength);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, PinholeRadialTangentialCameraModel::Fx);
    }

    // Test that optimizing for principal_points works correctly.
    indices = camera.fixedParameterIndices(_Type::PrincipalPoint);
    EXPECT_EQ(indices.size(), camera.numParameters() - 2);
    for (const auto& index : indices) {
        EXPECT_NE(index, PinholeRadialTangentialCameraModel::Cx);
        EXPECT_NE(index, PinholeRadialTangentialCameraModel::Cy);
    }

    // Test that optimizing for aspect ratio works correctly.
    indices = camera.fixedParameterIndices(_Type::AspectRatio);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, PinholeRadialTangentialCameraModel::YX);
    }

    // Test that optimizing for skew works correctly.
    indices = camera.fixedParameterIndices(_Type::Skew);
    EXPECT_EQ(indices.size(), camera.numParameters() - 1);
    for (const auto& index : indices) {
        EXPECT_NE(index, PinholeRadialTangentialCameraModel::Skew);
    }

    // Test that optimizing for radial distortion works correctly.
    indices = camera.fixedParameterIndices(_Type::RadialDistortion);
    EXPECT_EQ(indices.size(), camera.numParameters() - 3);
    for (const auto& index : indices) {
        EXPECT_NE(index, PinholeRadialTangentialCameraModel::K1);
        EXPECT_NE(index, PinholeRadialTangentialCameraModel::K2);
        EXPECT_NE(index, PinholeRadialTangentialCameraModel::K3);
    }

    // Test that optimizing for tangential distortion does not optimize any
    // parameters.
    indices = camera.fixedParameterIndices(_Type::TangentialDistortion);
    EXPECT_EQ(indices.size(), camera.numParameters() - 2);
    for (const auto& index : indices) {
        EXPECT_NE(index, PinholeRadialTangentialCameraModel::T1);
        EXPECT_NE(index, PinholeRadialTangentialCameraModel::T2);
    }
}

class PinholeRadialTangentialCameraFixture : public ::testing::Test
{
protected:
    void SetUp() override
    {
        constexpr double kPrincipalPoint[2]{600., 400.};
        constexpr double kFocalLength{1200.};

        _camera.setFocalLength(kFocalLength);
        _camera.setPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
    }

    void TestReprojection() const
    {
        constexpr double kTolerance = 1e-5;
        const double kNormalizedTolerance = kTolerance / _camera.focalLength();
        constexpr int kImageWidth = 1200;
        constexpr int kImageHeight = 980;
        constexpr double kMinDepth{2.0};
        constexpr double kMaxDepth{25.0};

        // Ensure the image -> camera -> image transformation works.
        for (double x = 0.0; x < kImageWidth; x += 10.0) {
            for (double y = 0.0; y < kImageHeight; y += 10.0) {
                const Vector2d pixel{x, y};
                const Vector3d normalized_ray = _camera.imageToSpace(pixel);

                // Test the reprojection at several depths.
                for (double depth = kMinDepth; depth < kMaxDepth;
                     depth += 1.0) {
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
                for (double depth = kMinDepth; depth < kMaxDepth;
                     depth += 1.0) {
                    const Vector3d point{x, y, depth};
                    const Vector2d pixel = _camera.spaceToImage(point);
                    const Vector3d normalized_ray = _camera.imageToSpace(pixel);
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
    PinholeRadialTangentialCameraModel _camera;
};

TEST_F(PinholeRadialTangentialCameraFixture, ReprojectionNoDistortion)
{
    _camera.setRadialDistortion(0., 0., 0.);
    TestReprojection();
}

TEST_F(PinholeRadialTangentialCameraFixture, ReprojectionOneRadialDistortion)
{
    _camera.setRadialDistortion(0.01, 0., 0.);
    TestReprojection();
}

TEST_F(PinholeRadialTangentialCameraFixture, ReprojectionTwoRadialDistortion)
{
    _camera.setRadialDistortion(0.01, 0.001, 0.);
    TestReprojection();
}

TEST_F(PinholeRadialTangentialCameraFixture,
       ReprojectionOneTangentialDistortion)
{
    _camera.setRadialDistortion(0.01, 0.001, 0.0001);
    _camera.setTangentialDistortion(0.01, 0.);
    TestReprojection();
}

TEST_F(PinholeRadialTangentialCameraFixture,
       ReprojectionTwoTangentialDistortion)
{
    _camera.setRadialDistortion(0.01, 0.001, 0.0001);
    _camera.setTangentialDistortion(0.01, 0.001);
    TestReprojection();
}
