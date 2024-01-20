#include "reprojection_error.h"

#include "division_undistortion_camera_model.h"
#include "double_sphere_camera_model.h"
#include "extended_unified_camera_model.h"
#include "fisheye_camera_model.h"
#include "fov_camera_model.h"
#include "pinhole_camera_model.h"
#include "pinhole_radial_tangential_camera_model.h"
#include "omnidirectional_camera_model.h"
#include "orthographic_camera_model.h"

namespace tl {

ceres::CostFunction* createReprojectionErrorCostFunction(
    CameraIntrinsics::Type type, const Eigen::Vector2d& feature)
{
    switch (type) {
        case CameraIntrinsics::Type::Pinhole:
            return ReprojectionError<PinholeCameraModel>::create(feature);
        case CameraIntrinsics::Type::PinholeRadialTangential:
            return ReprojectionError<
                PinholeRadialTangentialCameraModel>::create(feature);
        case CameraIntrinsics::Type::Fisheye:
            return ReprojectionError<FisheyeCameraModel>::create(feature);
        case CameraIntrinsics::Type::Fov:
            return ReprojectionError<FOVCameraModel>::create(feature);
        case CameraIntrinsics::Type::DivisionUndistortion:
            return ReprojectionError<DivisionUndistortionCameraModel>::create(
                feature);
        case CameraIntrinsics::Type::DoubleSphere:
            return ReprojectionError<DoubleSphereCameraModel>::create(feature);
        case CameraIntrinsics::Type::ExtendedUnified:
            return ReprojectionError<ExtendedUnifiedCameraModel>::create(
                feature);
        case CameraIntrinsics::Type::Omnidirectional:
            return ReprojectionError<OmnidirectionalCameraModel>::create(
                feature);
        case CameraIntrinsics::Type::Orthographic:
            return ReprojectionError<OrthographicCameraModel>::create(feature);
        default:
            LOG(FATAL) << "Unsupported camera type.";
            break;
    }

    return nullptr;
}

} // namespace tl
