#include "camera_intrinsics.h"

#include <tCore/Math>

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

std::ostream& operator<<(std::ostream& os, const CameraIntrinsics& cam)
{
    return os << cam.toLog();
}

} // namespace tl
