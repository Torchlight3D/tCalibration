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

template <class CameraModel>
struct InvReprojectionPoseError
{
public:
    explicit InvReprojectionPoseError(const Feature& feature,
                                      const Eigen::Vector3d& bearing_vector_ref)
        : feature_(feature), bearing_vector_ref_(bearing_vector_ref)
    {
    }

    template <typename T>
    bool operator()(const T* extrinsic_parameters_ref,
                    const T* intrinsic_parameters_ref, const T* inverse_depth,
                    T* reprojection_error) const
    {
        using namespace Eigen;

        // first scale the bearing vector
        Vector3<T> point_ref = bearing_vector_ref_.cast<T>() / *inverse_depth;

        // construct transformation matrices
        Sophus::SO3<T> R_ref_world = Sophus::SO3<T>::exp(Map<const Vector3<T>>(
            extrinsic_parameters_ref + Camera::Orientation));

        Vector3<T> p_world_ref =
            Map<const Vector3<T>>(extrinsic_parameters_ref + Camera::Position);

        Sophus::SE3<T> T_world_ref(R_ref_world.inverse(), p_world_ref);

        // transform the vector to the other camera
        Vector3<T> point = T_world_ref.inverse() * T_world_ref * point_ref;

        // Apply the camera intrinsics to get the reprojected pixel.
        T reprojection[2];
        const bool res = CameraModel::spaceToPixel(intrinsic_parameters_ref,
                                                   point.data(), reprojection);

        const T sqrt_information_x =
            T(1. / ceres::sqrt(feature_.covariance(0, 0)));
        const T sqrt_information_y =
            T(1. / ceres::sqrt(feature_.covariance(1, 1)));
        reprojection_error[0] =
            sqrt_information_x * (reprojection[0] - feature_.pos.x());
        reprojection_error[1] =
            sqrt_information_y * (reprojection[1] - feature_.pos.y());

        return res;
    }

private:
    const Feature feature_;
    const Eigen::Vector3d bearing_vector_ref_;
};

ceres::CostFunction* createInvReprojectionPoseErrorCostFunction(
    CameraIntrinsics::Type type, const Feature& feature,
    const Eigen::Vector3d& ref_bearing)
{
    constexpr int kResidualSize = 2;
    constexpr int kPointSize = 1;
    // Return the appropriate reprojection error cost function based on the
    // camera model type.
    switch (type) {
        case CameraIntrinsics::Type::Pinhole:
            return new ceres::AutoDiffCostFunction<
                InvReprojectionPoseError<PinholeCameraModel>, kResidualSize,
                Camera::ExtrinsicsSize, PinholeCameraModel::IntrinsicsSize,
                kPointSize>(new InvReprojectionPoseError<PinholeCameraModel>(
                feature, ref_bearing));
        case CameraIntrinsics::Type::PinholeRadialTangential:
            return new ceres::AutoDiffCostFunction<
                InvReprojectionPoseError<PinholeRadialTangentialCameraModel>,
                kResidualSize, Camera::ExtrinsicsSize,
                PinholeRadialTangentialCameraModel::IntrinsicsSize, kPointSize>(
                new InvReprojectionPoseError<
                    PinholeRadialTangentialCameraModel>(feature, ref_bearing));
        case CameraIntrinsics::Type::Fisheye:
            return new ceres::AutoDiffCostFunction<
                InvReprojectionPoseError<FisheyeCameraModel>, kResidualSize,
                Camera::ExtrinsicsSize, FisheyeCameraModel::IntrinsicsSize,
                kPointSize>(new InvReprojectionPoseError<FisheyeCameraModel>(
                feature, ref_bearing));
        case CameraIntrinsics::Type::Fov:
            return new ceres::AutoDiffCostFunction<
                InvReprojectionPoseError<FOVCameraModel>, kResidualSize,
                Camera::ExtrinsicsSize, FOVCameraModel::IntrinsicsSize,
                kPointSize>(new InvReprojectionPoseError<FOVCameraModel>(
                feature, ref_bearing));
        case CameraIntrinsics::Type::DivisionUndistortion:
            return new ceres::AutoDiffCostFunction<
                InvReprojectionPoseError<DivisionUndistortionCameraModel>,
                kResidualSize, Camera::ExtrinsicsSize,
                DivisionUndistortionCameraModel::IntrinsicsSize, kPointSize>(
                new InvReprojectionPoseError<DivisionUndistortionCameraModel>(
                    feature, ref_bearing));
        case CameraIntrinsics::Type::DoubleSphere:
            return new ceres::AutoDiffCostFunction<
                InvReprojectionPoseError<DoubleSphereCameraModel>,
                kResidualSize, Camera::ExtrinsicsSize,
                DoubleSphereCameraModel::IntrinsicsSize, kPointSize>(
                new InvReprojectionPoseError<DoubleSphereCameraModel>(
                    feature, ref_bearing));
        case CameraIntrinsics::Type::ExtendedUnified:
            return new ceres::AutoDiffCostFunction<
                InvReprojectionPoseError<ExtendedUnifiedCameraModel>,
                kResidualSize, Camera::ExtrinsicsSize,
                ExtendedUnifiedCameraModel::IntrinsicsSize, kPointSize>(
                new InvReprojectionPoseError<ExtendedUnifiedCameraModel>(
                    feature, ref_bearing));
        default:
            LOG(FATAL) << "Failed to create inverse reprojection pose error "
                          "cost function: "
                          "Unsupported camera type.";
            break;
    }

    return nullptr;
}

} // namespace tl
