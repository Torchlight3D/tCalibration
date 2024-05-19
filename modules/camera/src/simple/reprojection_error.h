#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "camera.h"

namespace tl {

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
                    const T* point, T* residual) const
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

        if (!CameraIntrinsics::isUnprojectable(intrinsics, reprojection)) {
            return false;
        }

        // Compute the reprojection error.
        residual[0] = reprojection[0] - feature_.x();
        residual[1] = reprojection[1] - feature_.y();
        return true;
    }

    static ceres::CostFunction* create(const Eigen::Vector2d& feature)
    {
        constexpr int kResidualSize{2};
        constexpr int kPointSize{4};
        return new ceres::AutoDiffCostFunction<
            ReprojectionError<CameraIntrinsics>, kResidualSize,
            Camera::ExtrinsicsSize, CameraIntrinsics::IntrinsicsSize,
            kPointSize>(new ReprojectionError<CameraIntrinsics>(feature));
    }
};

// TODO:
// 1. merge into Camera??
// 2. use CameraIntrinsics as template parameter
ceres::CostFunction* createReprojectionErrorCostFunction(
    CameraIntrinsics::Type type, const Eigen::Vector2d& feature);

} // namespace tl
