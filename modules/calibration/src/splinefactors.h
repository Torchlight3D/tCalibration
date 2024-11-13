#pragma once

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
#include <tMotion/ImuIntrinsics>
#include <tMotion/Spline/SplineBase>
#include <tMvs/Scene>
#include <tMvs/View>

namespace tl {

inline constexpr int kBiasSplineOrder = 3;

template <int _Order>
struct AccelerationCostFunctorSplit : public SplineBase<_Order>
{
    using Base = SplineBase<_Order>;

    enum
    {
        Order = Base::Order,
        Degree = Base::Degree,
    };

    Eigen::Vector3d measurement;
    double u_r3, inv_r3_dt;
    double u_so3, inv_so3_dt; // Could share the same timestamp?
    double inv_std;
    // bias spline
    double u_bias, inv_bias_dt;

    AccelerationCostFunctorSplit(const Eigen::Vector3d& measurement,
                                 double u_r3, double inv_r3_dt, double u_so3,
                                 double inv_so3_dt, double inv_std,
                                 double u_bias, double inv_bias_dt)
        : measurement(measurement),
          u_r3(u_r3),
          inv_r3_dt(inv_r3_dt),
          u_so3(u_so3),
          inv_so3_dt(inv_so3_dt),
          inv_std(inv_std),
          u_bias(u_bias),
          inv_bias_dt(inv_bias_dt)
    {
    }

    template <class T>
    bool operator()(const T* const* knots, T* residuals) const
    {
        using SO3 = Sophus::SO3<T>;
        using Vector3 = Eigen::Vector3<T>;
        using Vector6 = Eigen::Vector<T, 6>;

        SO3 R_wi;
        this->template evaluate_lie<T, Sophus::SO3>(knots, T(u_so3),
                                                    T(inv_so3_dt), &R_wi);

        Vector3 accel_w;
        this->template evaluate<T, 3, 2>(knots + Order, T(u_r3), T(inv_r3_dt),
                                         &accel_w);

        Vector3 bias_spline;
        SplineBase<kBiasSplineOrder>::evaluate<T, 3, 0>(
            knots + 2 * Order, T(u_bias), T(inv_bias_dt), &bias_spline);

        const Eigen::Map<const Vector3> gravity{
            knots[2 * Order + kBiasSplineOrder]};
        const Eigen::Map<const Vector6> acl_intrs{
            knots[2 * Order + kBiasSplineOrder + 1]};

        ImuIntrinsics_<T> intrinsics(acl_intrs[0], acl_intrs[1], acl_intrs[2],
                                     T(0), T(0), T(0), acl_intrs[3],
                                     acl_intrs[4], acl_intrs[5], bias_spline[0],
                                     bias_spline[1], bias_spline[2]);

        Vector3 accl_raw = measurement.cast<T>();
        Eigen::Map<Vector3>{residuals} =
            T(inv_std) * (R_wi.inverse() * (accel_w + gravity) -
                          intrinsics.unbiasNormalize(accl_raw));
        return true;
    }
};

template <int _Order, template <class, int = 0> class _LieGroup>
struct GyroCostFunctorSplit : public SplineBase<_Order>
{
    using Base = SplineBase<_Order>;

    enum
    {
        Order = Base::Order,
        Degree = Base::Degree,
    };

    Eigen::Vector3d measurement;
    double u_so3, inv_so3_dt;
    double inv_std;
    // bias
    double u_bias, inv_bias_dt;

    GyroCostFunctorSplit(const Eigen::Vector3d& measurement, double u_so3,
                         double inv_so3_dt, double inv_std, double u_bias,
                         double inv_bias_dt)
        : measurement(measurement),
          u_so3(u_so3),
          inv_so3_dt(inv_so3_dt),
          inv_std(inv_std),
          u_bias(u_bias),
          inv_bias_dt(inv_bias_dt)
    {
    }

    template <class T>
    bool operator()(const T* const* knots, T* residuals) const
    {
        using Tangent = typename _LieGroup<T>::Tangent;
        using Vector9 = Eigen::Vector<T, 9>;
        using Vector3 = Eigen::Vector3<T>;

        Tangent rot_vel;
        this->template evaluate_lie<T, _LieGroup>(
            knots, T(u_so3), T(inv_so3_dt), nullptr, &rot_vel);

        Vector3 bias_spline;
        SplineBase<kBiasSplineOrder>::evaluate<T, 3, 0>(
            knots + Order, T(u_bias), T(inv_bias_dt), &bias_spline);

        const Eigen::Map<const Vector9> gyr_intrs(
            knots[Order + kBiasSplineOrder]);
        ImuIntrinsics_<T> intrinsics(
            gyr_intrs[0], gyr_intrs[1], gyr_intrs[2], gyr_intrs[3],
            gyr_intrs[4], gyr_intrs[5], gyr_intrs[6], gyr_intrs[7],
            gyr_intrs[8], bias_spline[0], bias_spline[1], bias_spline[2]);

        Vector3 gyro_raw = measurement.cast<T>();
        Tangent tang(intrinsics.unbiasNormalize(gyro_raw));
        Eigen::Map<Tangent>{residuals} = T(inv_std) * (rot_vel - tang);

        return true;
    }
};

template <int _Order>
struct GSReprojectionCostFunctorSplit : public SplineBase<_Order>
{
    using Base = SplineBase<_Order>;

    enum
    {
        Order = Base::Order,
        Degree = Base::Degree,
    };

    const View* const _view;
    const Scene* const _scene;
    const std::vector<TrackId> _trackIds;
    const double u_so3, inv_so3_dt;
    const double u_r3, inv_r3_dt;

    GSReprojectionCostFunctorSplit(const View* view, const Scene* scene,
                                   double u_so3, double u_r3, double inv_so3_dt,
                                   double inv_r3_dt,
                                   const std::vector<TrackId>& trackIds)
        : _view(view),
          _scene(scene),
          u_so3(u_so3),
          u_r3(u_r3),
          inv_so3_dt(inv_so3_dt),
          inv_r3_dt(inv_r3_dt),
          _trackIds(trackIds)
    {
    }

    template <class T>
    bool operator()(const T* const* knots, T* residuals) const
    {
        using SE3 = Sophus::SE3<T>;
        using SO3 = Sophus::SO3<T>;
        using Matrix4 = Eigen::Matrix4<T>;
        using Vector3 = Eigen::Vector3<T>;
        using Vector4 = Eigen::Vector4<T>;

        const int N2 = 2 * Order;
        const Eigen::Map<const SE3> T_ic{knots[N2]};

        const auto cam = _view->camera();
        const auto cam_model = cam.cameraIntrinsicsModel();

        T intr[10];
        for (int i = 0; i < cam.cameraIntrinsics()->numParameters(); ++i) {
            intr[i] = T(cam.intrinsics()[i]);
        }

        SO3 R_wi;
        this->template evaluate_lie<T, Sophus::SO3>(knots, T(u_so3),
                                                    T(inv_so3_dt), &R_wi);

        Vector3 t_wi;
        this->template evaluate<T, 3, 0>(knots + Order, T(u_r3), T(inv_r3_dt),
                                         &t_wi);

        const SE3 T_wc = SE3{R_wi, t_wi} * T_ic;
        const Matrix4 T_cw = T_wc.inverse().matrix();

        for (size_t i{0}; i < _trackIds.size(); ++i) {
            const auto feature = *_view->featureOf(_trackIds[i]);

            // get corresponding 3d point
            const Eigen::Map<const Vector4> scene_point(knots[N2 + i]);

            Vector3 p3d = (T_cw * scene_point).hnormalized();

            T reprojection[2];
            bool success = false;
            if (CameraIntrinsicsType::DivisionUndistortion == cam_model) {
                success = DivisionUndistortionCameraModel::spaceToPixel(
                    intr, p3d.data(), reprojection);
            }
            else if (CameraIntrinsicsType::DoubleSphere == cam_model) {
                success = DoubleSphereCameraModel::spaceToPixel(
                    intr, p3d.data(), reprojection);
            }
            else if (CameraIntrinsicsType::Pinhole == cam_model) {
                success = PinholeCameraModel::spaceToPixel(intr, p3d.data(),
                                                           reprojection);
            }
            else if (CameraIntrinsicsType::Fisheye == cam_model) {
                success = FisheyeCameraModel::spaceToPixel(intr, p3d.data(),
                                                           reprojection);
            }
            else if (CameraIntrinsicsType::ExtendedUnified == cam_model) {
                success = ExtendedUnifiedCameraModel::spaceToPixel(
                    intr, p3d.data(), reprojection);
            }
            else if (CameraIntrinsicsType::PinholeRadialTangential ==
                     cam_model) {
                success = PinholeRadialTangentialCameraModel::spaceToPixel(
                    intr, p3d.data(), reprojection);
            }

            if (!success) {
                residuals[2 * i + 0] = T(1e10);
                residuals[2 * i + 1] = T(1e10);
            }
            else {
                const T inv_info_x = T(1. / sqrt(feature.covariance(0, 0)));
                const T inv_info_y = T(1. / sqrt(feature.covariance(1, 1)));
                residuals[2 * i + 0] =
                    inv_info_x * (reprojection[0] - T(feature.x()));
                residuals[2 * i + 1] =
                    inv_info_y * (reprojection[1] - T(feature.y()));
            }
        }
        return true;
    }
};

template <int _Order>
struct RSReprojectionCostFunctorSplit : public SplineBase<_Order>
{
    using Base = SplineBase<_Order>;

    enum
    {
        Order = Base::Order,
        Degree = Base::Degree,
    };

    const View* _view;
    const Scene* _scene;
    const std::vector<TrackId> _trackIds;
    const double u_so3, inv_so3_dt;
    const double u_r3, inv_r3_dt;

    RSReprojectionCostFunctorSplit(const View* view, const Scene* scene,
                                   double u_so3, double u_r3, double inv_so3_dt,
                                   double inv_r3_dt,
                                   const std::vector<TrackId>& trackIds)
        : _view(view),
          _scene(scene),
          u_so3(u_so3),
          u_r3(u_r3),
          inv_so3_dt(inv_so3_dt),
          inv_r3_dt(inv_r3_dt),
          _trackIds(trackIds)
    {
    }

    template <class T>
    bool operator()(const T* const* knots, T* residuals) const
    {
        using SE3 = Sophus::SE3<T>;
        using SO3 = Sophus::SO3<T>;
        using Matrix4 = Eigen::Matrix4<T>;
        using Vector1 = Eigen::Vector<T, 1>;
        using Vector3 = Eigen::Vector3<T>;
        using Vector4 = Eigen::Vector4<T>;

        constexpr int N2 = 2 * Order;
        const Eigen::Map<const SE3> T_i_c(knots[N2]);
        const Eigen::Map<const Vector1> line_delay(knots[N2 + 1]);

        const auto cam = _view->camera();
        const auto cam_model = cam.cameraIntrinsicsModel();

        T intr[10];
        for (int i = 0; i < cam.cameraIntrinsics()->numParameters(); ++i) {
            intr[i] = T(cam.intrinsics()[i]);
        }

        // if we have a rolling shutter cam we will always need to evaluate with
        // line delay
        for (size_t i = 0; i < _trackIds.size(); ++i) {
            const auto feature = *_view->featureOf(_trackIds[i]);

            // get time for respective RS line
            const auto y_coord = T(feature.y()) * line_delay[0];
            const auto t_so3_row = T(u_so3) + y_coord;
            const auto t_r3_row = T(u_r3) + y_coord;

            SO3 R_wi;
            this->template evaluate_lie<T, Sophus::SO3>(knots, t_so3_row,
                                                        T(inv_so3_dt), &R_wi);

            Vector3 t_wi;
            this->template evaluate<T, 3, 0>(knots + Order, t_r3_row,
                                             T(inv_r3_dt), &t_wi);

            const SE3 T_wc = SE3{R_wi, t_wi} * T_i_c;
            const Matrix4 T_cw = T_wc.inverse().matrix();

            // get corresponding 3d point
            const Eigen::Map<const Vector4> scene_point(knots[N2 + 2 + i]);

            Vector3 p3d = (T_cw * scene_point).hnormalized();

            T reprojection[2];
            bool success = false;
            if (CameraIntrinsicsType::DivisionUndistortion == cam_model) {
                success = DivisionUndistortionCameraModel::spaceToPixel(
                    intr, p3d.data(), reprojection);
            }
            else if (CameraIntrinsicsType::DoubleSphere == cam_model) {
                success = DoubleSphereCameraModel::spaceToPixel(
                    intr, p3d.data(), reprojection);
            }
            else if (CameraIntrinsicsType::Pinhole == cam_model) {
                success = PinholeCameraModel::spaceToPixel(intr, p3d.data(),
                                                           reprojection);
            }
            else if (CameraIntrinsicsType::Fisheye == cam_model) {
                success = FisheyeCameraModel::spaceToPixel(intr, p3d.data(),
                                                           reprojection);
            }
            else if (CameraIntrinsicsType::ExtendedUnified == cam_model) {
                success = ExtendedUnifiedCameraModel::spaceToPixel(
                    intr, p3d.data(), reprojection);
            }
            else if (CameraIntrinsicsType::PinholeRadialTangential ==
                     cam_model) {
                success = PinholeRadialTangentialCameraModel::spaceToPixel(
                    intr, p3d.data(), reprojection);
            }

            if (!success) {
                residuals[2 * i + 0] = T(1e10);
                residuals[2 * i + 1] = T(1e10);
            }
            else {
                const T inv_info_x = T(1. / sqrt(feature.covariance(0, 0)));
                const T inv_info_y = T(1. / sqrt(feature.covariance(1, 1)));
                residuals[2 * i + 0] =
                    inv_info_x * (reprojection[0] - T(feature.x()));
                residuals[2 * i + 1] =
                    inv_info_y * (reprojection[1] - T(feature.y()));
            }
        }

        return true;
    }
};

} // namespace tl
