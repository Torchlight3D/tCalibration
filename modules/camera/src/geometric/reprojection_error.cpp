#include "reprojection_error.h"

#include <ceres/rotation.h>

#include "camera.h"
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

using Eigen::Vector2d;
using Eigen::Vector3d;

template <class CameraIntrinsics>
struct ReprojectionError
{
    const Eigen::Vector2d feature_;

    explicit ReprojectionError(const Eigen::Vector2d& feature)
        : feature_(feature)
    {
    }

    template <typename T>
    bool operator()(const T* const extrinsics, const T* const intrinsics,
                    const T* point, T* residuals) const
    {
        using ConstVec3_t = Eigen::Map<const Eigen::Vector3<T>>;

        // Remove the translation.
        Eigen::Vector3<T> adjusted_point =
            ConstVec3_t(point) -
            point[3] * ConstVec3_t(extrinsics + Camera::Position);

        // If the point is too close to the camera center then the point cannot
        // be constrained by triangulation. This is likely to only occur when a
        // 3d point is seen by 2 views and the camera center of 1 view lies on
        // or neare the optical axis of the other view.
        //
        // Since we do not know the camera model we cannot say that the point
        // must be in front of the camera (e.g., wide angle cameras that have >
        // 180 degree FOV). Instead we simply force that the point is not near
        // the camera center.
        static const T kVerySmallNumber(1e-8);
        if (adjusted_point.squaredNorm() < kVerySmallNumber) {
            return false;
        }

        // Rotate the point to obtain the point in the camera coordinate system.
        T rotated_point[3];
        ceres::AngleAxisRotatePoint(extrinsics + Camera::Orientation,
                                    adjusted_point.data(), rotated_point);

        // Apply the camera intrinsics to get the reprojected pixel.
        T reprojection[2];
        CameraIntrinsics::spaceToPixel(intrinsics, rotated_point, reprojection);

        // Compute the reprojection error.
        residuals[0] = reprojection[0] - feature_.x();
        residuals[1] = reprojection[1] - feature_.y();
        return true;
    }

    static ceres::CostFunction* create(const Eigen::Vector2d& feature)
    {
        return new ceres::AutoDiffCostFunction<
            ReprojectionError<CameraIntrinsics>, Vector2d::SizeAtCompileTime,
            Camera::ExtrinsicsSize, CameraIntrinsics::IntrinsicsSize,
            Vector3d::HomogeneousReturnType::SizeAtCompileTime>(
            new ReprojectionError<CameraIntrinsics>(feature));
    }
};

ceres::CostFunction* createReprojectionErrorCostFunction(
    CameraIntrinsicsType type, const Eigen::Vector2d& feature)
{
    switch (type) {
        case CameraIntrinsicsType::Pinhole:
            return ReprojectionError<PinholeCameraModel>::create(feature);
        case CameraIntrinsicsType::PinholeRadialTangential:
            return ReprojectionError<
                PinholeRadialTangentialCameraModel>::create(feature);
        case CameraIntrinsicsType::Fisheye:
            return ReprojectionError<FisheyeCameraModel>::create(feature);
        case CameraIntrinsicsType::Fov:
            return ReprojectionError<FovCameraModel>::create(feature);
        case CameraIntrinsicsType::DivisionUndistortion:
            return ReprojectionError<DivisionUndistortionCameraModel>::create(
                feature);
        case CameraIntrinsicsType::DoubleSphere:
            return ReprojectionError<DoubleSphereCameraModel>::create(feature);
        case CameraIntrinsicsType::ExtendedUnified:
            return ReprojectionError<ExtendedUnifiedCameraModel>::create(
                feature);
        case CameraIntrinsicsType::Omnidirectional:
            return ReprojectionError<OmnidirectionalCameraModel>::create(
                feature);
        case CameraIntrinsicsType::Orthographic:
            return ReprojectionError<OrthographicCameraModel>::create(feature);
        default:
            LOG(FATAL) << "Unsupported camera type.";
            break;
    }

    return nullptr;
}

} // namespace tl
