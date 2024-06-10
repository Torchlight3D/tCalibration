#include "camera_intrinsics.h"

#include <tCore/Math>

#include "../util_camera_matrix.h"

#include "division_undistortion_camera_model.h"
#include "double_sphere_camera_model.h"
#include "extended_unified_camera_model.h"
#include "fisheye_camera_model.h"
#include "fov_camera_model.h"
#include "omnidirectional_camera_model.h"
#include "orthographic_camera_model.h"
#include "pinhole_camera_model.h"
#include "pinhole_radial_tangential_camera_model.h"

namespace tl {

using Eigen::MatrixXf;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector2i;
using Eigen::Vector3d;

CameraIntrinsics::Ptr CameraIntrinsics::create(CameraIntrinsicsType type)
{
    switch (type) {
        using Type = CameraIntrinsicsType;

        case Type::Pinhole:
            return std::make_shared<PinholeCameraModel>();
        case Type::PinholeRadialTangential:
            return std::make_shared<PinholeRadialTangentialCameraModel>();
        case Type::Fisheye:
            return std::make_shared<FisheyeCameraModel>();
        case Type::Fov:
            return std::make_shared<FovCameraModel>();
        case Type::DivisionUndistortion:
            return std::make_shared<DivisionUndistortionCameraModel>();
        case Type::DoubleSphere:
            return std::make_shared<DoubleSphereCameraModel>();
        case Type::ExtendedUnified:
            return std::make_shared<ExtendedUnifiedCameraModel>();
        case Type::Omnidirectional:
            return std::make_shared<OmnidirectionalCameraModel>();
        case Type::Orthographic:
            return std::make_shared<OrthographicCameraModel>();
        default:
            LOG(FATAL) << "Unsupported Camera model.";
            break;
    }

    return nullptr;
}

void CameraIntrinsics::setFromMetaData(const CameraMetaData& meta)
{
    // Focal length
    if (meta.focalLength.has_value()) {
        setFocalLength(meta.focalLength.value()[0]);
    }
    else if (meta.imageSize.has_value()) {
        constexpr auto kEmpiricalScale{1.2};
        setFocalLength(kEmpiricalScale *
                       std::max(meta.imageWidth(), meta.imageHeight()));
    }

    if (meta.aspectRatio.has_value()) {
        setAspectRatio(meta.aspectRatio.value()[0]);
    }

    // Principal point
    if (meta.principalPoint.has_value()) {
        setPrincipalPoint(meta.cx(), meta.cy());
    }
    else if (meta.imageSize.has_value()) {
        setPrincipalPoint(meta.imageWidth() / 2., meta.imageHeight() / 2.);
    }
}

CameraMetaData CameraIntrinsics::toMetaData() const
{
    CameraMetaData meta;
    meta.intrinsicType = magic_enum::enum_name(type());
    meta.focalLength = {focalLength()};
    meta.aspectRatio = {aspectRatio()};
    meta.principalPoint = {cx(), cy()};

    return meta;
}

void CameraIntrinsics::setFocalLength(double fx)
{
    CHECK_GT(fx, 0.) << "Invalid focal length value."
                        "Focal length must be greater than 0.0.";
    setParameter(Fx, fx);
}

double CameraIntrinsics::focalLength() const { return parameter(Fx); }

void CameraIntrinsics::setAspectRatio(double aspectRatio)
{
    CHECK_GT(aspectRatio, 0.0) << "Invalid aspect ratio."
                                  "Aspect ratio must be greater than 0.0.";
    setParameter(YX, aspectRatio);
}

double CameraIntrinsics::aspectRatio() const { return parameter(YX); }

void CameraIntrinsics::setPrincipalPoint(double cx, double cy)
{
    setParameter(Cx, cx);
    setParameter(Cy, cy);
}

double CameraIntrinsics::principalPointX() const { return parameter(Cx); }

double CameraIntrinsics::principalPointY() const { return parameter(Cy); }

std::vector<int> CameraIntrinsics::fixedParameterIndices(
    OptimizeIntrinsicsType flags) const
{
    using Type = OptimizeIntrinsicsType;

    if (flags == Type::All) {
        return {};
    }

    std::vector<int> indices;
    if ((flags & Type::FocalLength) == Type::None) {
        indices.emplace_back(Fx);
    }
    if ((flags & Type::AspectRatio) == Type::None) {
        indices.emplace_back(YX);
    }
    if ((flags & Type::PrincipalPoint) == Type::None) {
        indices.emplace_back(Cx);
        indices.emplace_back(Cy);
    }

    return indices;
}

Eigen::Matrix3d CameraIntrinsics::calibrationMatrix() const
{
    return intrinsicsToCalibrationMatrix(fx(), 0., aspectRatio(), cx(), cy());
}

void CameraIntrinsics::scale(double s)
{
    setFocalLength(fx() * s);
    setPrincipalPoint(s * (cx() + 0.5) - 0.5, s * (cy() + 0.5) - 0.5);
}

std::ostream& operator<<(std::ostream& os, const CameraIntrinsics& cam)
{
    return os << cam.toLog();
}

} // namespace tl
