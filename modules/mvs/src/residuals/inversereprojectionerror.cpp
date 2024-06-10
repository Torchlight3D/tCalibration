#include "inversereprojectionerror.h"

#include <ceres/autodiff_cost_function.h>
#include <ceres/jet.h>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <tCamera/DivisionUndistortionCameraModel>
#include <tCamera/DoubleSphereCameraModel>
#include <tCamera/ExtendedUnifiedCameraModel>
#include <tCamera/FisheyeCameraModel>
#include <tCamera/FovCameraModel>
#include <tCamera/OmnidirectionalCameraModel>
#include <tCamera/OrthographicCameraModel>
#include <tCamera/PinholeCameraModel>
#include <tCamera/PinholeRadialTangentialCameraModel>

namespace tl {

using Eigen::Vector2d;

template <class CameraIntrinsics>
struct InvReprojectionPoseError
{
    const Feature feature_;
    const Eigen::Vector3d bearing_vector_ref_;

    explicit InvReprojectionPoseError(const Feature& feature,
                                      const Eigen::Vector3d& bearing_vector_ref)
        : feature_(feature), bearing_vector_ref_(bearing_vector_ref)
    {
    }

    template <typename T>
    bool operator()(const T* extrinsics, const T* intrinsics,
                    const T* inverse_depth, T* residuals) const
    {
        using Vec3 = Eigen::Vector3<T>;
        using SO3 = Sophus::SO3<T>;
        using SE3 = Sophus::SE3<T>;

        // first scale the bearing vector
        const Vec3 point_ref = bearing_vector_ref_.cast<T>() / *inverse_depth;

        // construct transformation matrices
        const SO3 R_ref_world =
            SO3::exp(Eigen::Map<const Vec3>{extrinsics + Camera::Orientation});

        const Eigen::Map<const Vec3> p_world_ref{extrinsics + Camera::Position};

        const SE3 T_world_ref(R_ref_world.inverse(), p_world_ref);

        // transform the vector to the other camera
        const Vec3 point = T_world_ref.inverse() * T_world_ref * point_ref;

        // Apply the camera intrinsics to get the reprojected pixel.
        T reproj[2];
        const bool res =
            CameraIntrinsics::spaceToPixel(intrinsics, point.data(), reproj);

        const T sqrt_information_x = T(1. / sqrt(feature_.covariance(0, 0)));
        const T sqrt_information_y = T(1. / sqrt(feature_.covariance(1, 1)));
        residuals[0] = sqrt_information_x * (reproj[0] - feature_.pos.x());
        residuals[1] = sqrt_information_y * (reproj[1] - feature_.pos.y());

        return res;
    }

    static ceres::CostFunction* create(const Feature& feature,
                                       const Eigen::Vector3d& bearing)
    {
        constexpr int kPointSize = 1;

        return new ceres::AutoDiffCostFunction<
            InvReprojectionPoseError<CameraIntrinsics>,
            Vector2d::SizeAtCompileTime, Camera::ExtrinsicsSize,
            CameraIntrinsics::IntrinsicsSize, kPointSize>(
            new InvReprojectionPoseError<CameraIntrinsics>(feature, bearing));
    }
};

ceres::CostFunction* createInvReprojectionPoseErrorCostFunction(
    CameraIntrinsicsType type, const Feature& feature,
    const Eigen::Vector3d& ref_bearing)
{
    switch (type) {
        case CameraIntrinsicsType::Pinhole:
            return InvReprojectionPoseError<PinholeCameraModel>::create(
                feature, ref_bearing);
        case CameraIntrinsicsType::PinholeRadialTangential:
            return InvReprojectionPoseError<
                PinholeRadialTangentialCameraModel>::create(feature,
                                                            ref_bearing);
        case CameraIntrinsicsType::Fisheye:
            return InvReprojectionPoseError<FisheyeCameraModel>::create(
                feature, ref_bearing);
        case CameraIntrinsicsType::Fov:
            return InvReprojectionPoseError<FovCameraModel>::create(
                feature, ref_bearing);
        case CameraIntrinsicsType::DivisionUndistortion:
            return InvReprojectionPoseError<
                DivisionUndistortionCameraModel>::create(feature, ref_bearing);
        case CameraIntrinsicsType::DoubleSphere:
            return InvReprojectionPoseError<DoubleSphereCameraModel>::create(
                feature, ref_bearing);
        case CameraIntrinsicsType::ExtendedUnified:
            return InvReprojectionPoseError<ExtendedUnifiedCameraModel>::create(
                feature, ref_bearing);
        case CameraIntrinsicsType::Omnidirectional:
            return InvReprojectionPoseError<OmnidirectionalCameraModel>::create(
                feature, ref_bearing);
        case CameraIntrinsicsType::Orthographic:
            return InvReprojectionPoseError<OrthographicCameraModel>::create(
                feature, ref_bearing);
        default:
            LOG(FATAL) << "Unsupported camera type.";
            break;
    }

    return nullptr;
}

} // namespace tl
