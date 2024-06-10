#pragma once

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <tCamera/DivisionUndistortionCameraModel>
#include <tCamera/DoubleSphereCameraModel>
#include <tCamera/ExtendedUnifiedCameraModel>
#include <tCamera/FisheyeCameraModel>
#include <tCamera/OmnidirectionalCameraModel>
#include <tCamera/OrthographicCameraModel>
#include <tCamera/PinholeCameraModel>
#include <tCamera/PinholeRadialTangentialCameraModel>
#include <tMotion/ImuIntrinsics>
#include <tMvs/Scene>
#include <tMvs/View>

#include "ceres_spline_helper.h"

namespace tl {

inline constexpr int kBiasSplineOrder{3};

template <int _N>
struct AccelerationCostFunctorSplit : public CeresSplineHelper<double, _N>
{
    static constexpr int Order = _N;
    static constexpr int Degree = _N - 1;

    using MatN = Eigen::Matrix<double, _N, _N>;
    using VecN = Eigen::Matrix<double, _N, 1>;
    using Vec3d = Eigen::Vector3d;
    using Mat3d = Eigen::Matrix3d;

    Eigen::Vector3d measurement;
    double u_r3;
    double u_so3;
    double inv_r3_dt;
    double inv_so3_dt;
    double inv_std;
    // bias spline
    double u_bias;
    double inv_bias_dt;

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
    bool operator()(const T* const* sKnots, T* sResiduals) const
    {
        using Vector3 = Eigen::Vector3<T>;
        using Vector6 = Eigen::Matrix<T, 6, 1>;

        Sophus::SO3<T> R_w_i;
        CeresSplineHelper<T, Order>::template evaluate_lie<Sophus::SO3>(
            sKnots, T(u_so3), T(inv_so3_dt), &R_w_i);

        Vector3 accel_w;
        CeresSplineHelper<T, Order>::template evaluate<3, 2>(
            sKnots + Order, T(u_r3), T(inv_r3_dt), &accel_w);

        Vector3 bias_spline;
        CeresSplineHelper<T, kBiasSplineOrder>::template evaluate<3, 0>(
            sKnots + 2 * Order, T(u_bias), T(inv_bias_dt), &bias_spline);

        const Eigen::Map<const Vector3> gravity(
            sKnots[2 * Order + kBiasSplineOrder]);
        const Eigen::Map<const Vector6> acc_params(
            sKnots[2 * Order + kBiasSplineOrder + 1]);

        ImuIntrinsics_<T> acc_intrinsics{
            acc_params[0], acc_params[1],  acc_params[2],  T(0),
            T(0),          T(0),           acc_params[3],  acc_params[4],
            acc_params[5], bias_spline[0], bias_spline[1], bias_spline[2]};

        Eigen::Map<Vector3> residuals(sResiduals);
        Vector3 acc_raw{T(measurement[0]), T(measurement[1]),
                        T(measurement[2])};
        residuals = T(inv_std) * (R_w_i.inverse() * (accel_w + gravity) -
                                  acc_intrinsics.unbiasNormalize(acc_raw));
        return true;
    }
};

// FIXME: namespace confusion template<S, Opt = 0> vs. template<S, Opt>
template <int _N, template <class S, int> class GroupT, bool OLD_TIME_DERIV>
struct GyroCostFunctorSplit : public CeresSplineHelper<double, _N>
{
    static constexpr int N = _N;       // Order of the spline.
    static constexpr int DEG = _N - 1; // Degree of the spline.

    using MatN = Eigen::Matrix<double, _N, _N>;
    using VecN = Eigen::Matrix<double, _N, 1>;
    using Vec3 = Eigen::Vector3d;
    using Mat3 = Eigen::Matrix3d;

    using Tangentd = typename GroupT<double, 0>::Tangent;

    Eigen::Vector3d measurement;
    double u_so3;
    double inv_std;
    double inv_so3_dt;
    // bias
    double u_bias, inv_std_bias;
    double inv_bias_dt;

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
    bool operator()(const T* const* sKnots, T* sResiduals) const
    {
        using Tangent = typename GroupT<T, 0>::Tangent;
        using Vector9 = Eigen::Matrix<T, 9, 1>;
        using Vector3 = Eigen::Vector3<T>;

        Tangent rot_vel;

        CeresSplineHelper<T, N>::template evaluate_lie<GroupT>(
            sKnots, T(u_so3), T(inv_so3_dt), nullptr, &rot_vel);

        Vector3 bias_spline;
        CeresSplineHelper<T, kBiasSplineOrder>::template evaluate<3, 0>(
            sKnots + N, T(u_bias), T(inv_bias_dt), &bias_spline);

        const Eigen::Map<const Vector9> gyro_params(
            sKnots[N + kBiasSplineOrder]);
        ImuIntrinsics_<T> gyro_intrinsics{
            gyro_params[0], gyro_params[1], gyro_params[2], gyro_params[3],
            gyro_params[4], gyro_params[5], gyro_params[6], gyro_params[7],
            gyro_params[8], bias_spline[0], bias_spline[1], bias_spline[2]};

        Eigen::Map<Tangent> residuals(sResiduals);
        Vector3 gyro_raw{T(measurement[0]), T(measurement[1]),
                         T(measurement[2])};
        Tangent tang(gyro_intrinsics.unbiasNormalize(gyro_raw));
        residuals = T(inv_std) * (rot_vel - tang);

        return true;
    }
};

template <int _N>
struct GSReprojectionCostFunctorSplit : public CeresSplineHelper<double, _N>
{
    static constexpr int N = _N;       // Order of the spline.
    static constexpr int DEG = _N - 1; // Degree of the spline.

    using MatN = Eigen::Matrix<double, _N, _N>;
    using VecN = Eigen::Matrix<double, _N, 1>;
    using Vec3 = Eigen::Vector3d;
    using Mat3 = Eigen::Matrix3d;

    const View* view;
    const Scene* scene;
    std::vector<int> track_ids;
    double u_so3;
    double inv_so3_dt;
    double u_r3;
    double inv_r3_dt;

    GSReprojectionCostFunctorSplit(const View* view, const Scene* scene,
                                   double u_so3, double u_r3, double inv_so3_dt,
                                   double inv_r3_dt,
                                   const std::vector<int>& track_ids)
        : view(view),
          scene(scene),
          u_so3(u_so3),
          u_r3(u_r3),
          inv_so3_dt(inv_so3_dt),
          inv_r3_dt(inv_r3_dt),
          track_ids(track_ids)
    {
    }

    template <class T>
    bool operator()(const T* const* sKnots, T* sResiduals) const
    {
        using Vector3 = Eigen::Vector3<T>;
        using Vector4 = Eigen::Vector4<T>;
        using Matrix4 = Eigen::Matrix4<T>;

        const int N2 = 2 * N;
        Eigen::Map<const Sophus::SE3<T>> const T_i_c(sKnots[N2]);

        const auto cam = view->camera();
        const auto cam_model = cam.cameraIntrinsicsModel();

        T intr[10];
        for (int i = 0; i < cam.cameraIntrinsics()->numParameters(); ++i) {
            intr[i] = T(cam.intrinsics()[i]);
        }

        const T t_so3_row = T(u_so3);
        const T t_r3_row = T(u_r3);

        Sophus::SO3<T> R_w_i;
        CeresSplineHelper<T, N>::template evaluate_lie<Sophus::SO3>(
            sKnots, t_so3_row, T(inv_so3_dt), &R_w_i);

        Vector3 t_w_i;
        CeresSplineHelper<T, N>::template evaluate<3, 0>(sKnots + N, t_r3_row,
                                                         T(inv_r3_dt), &t_w_i);

        Sophus::SE3<T> T_w_c = Sophus::SE3<T>(R_w_i, t_w_i) * T_i_c;
        Matrix4 T_c_w_matrix = T_w_c.inverse().matrix();

        for (size_t i = 0; i < track_ids.size(); ++i) {
            const auto feature = *view->featureOf(track_ids[i]);

            // get corresponding 3d point
            const Eigen::Map<const Vector4> scene_point(sKnots[N2 + i]);

            Vector3 p3d = (T_c_w_matrix * scene_point).hnormalized();

            // TODO: Use auto registration
            T reprojection[2];
            bool success{false};
            switch (cam_model) {
                case CameraIntrinsicsType::DivisionUndistortion:
                    success = DivisionUndistortionCameraModel::spaceToPixel(
                        intr, p3d.data(), reprojection);
                    break;
                case CameraIntrinsicsType::DoubleSphere:
                    success = DoubleSphereCameraModel::spaceToPixel(
                        intr, p3d.data(), reprojection);
                    break;
                case CameraIntrinsicsType::Pinhole:
                    success = PinholeCameraModel::spaceToPixel(intr, p3d.data(),
                                                               reprojection);
                    break;
                case CameraIntrinsicsType::PinholeRadialTangential:
                    success = PinholeRadialTangentialCameraModel::spaceToPixel(
                        intr, p3d.data(), reprojection);
                    break;
                case CameraIntrinsicsType::Fisheye:
                    success = FisheyeCameraModel::spaceToPixel(intr, p3d.data(),
                                                               reprojection);
                    break;
                case CameraIntrinsicsType::ExtendedUnified:
                    success = ExtendedUnifiedCameraModel::spaceToPixel(
                        intr, p3d.data(), reprojection);
                    break;
                case CameraIntrinsicsType::Omnidirectional:
                    success = OmnidirectionalCameraModel::spaceToPixel(
                        intr, p3d.data(), reprojection);
                    break;
                case CameraIntrinsicsType::Orthographic:
                    success = OrthographicCameraModel::spaceToPixel(
                        intr, p3d.data(), reprojection);
                    break;
                default:
                    break;
            }

            if (!success) {
                sResiduals[2 * i + 0] = T(1e10);
                sResiduals[2 * i + 1] = T(1e10);
            }
            else {
                const T inv_info_x =
                    T(1. / ceres::sqrt(feature.covariance(0, 0)));
                const T inv_info_y =
                    T(1. / ceres::sqrt(feature.covariance(1, 1)));
                sResiduals[2 * i + 0] =
                    inv_info_x * (reprojection[0] - T(feature.x()));
                sResiduals[2 * i + 1] =
                    inv_info_y * (reprojection[1] - T(feature.y()));
            }
        }

        return true;
    }
};

template <int _N>
struct RSReprojectionCostFunctorSplit : public CeresSplineHelper<double, _N>
{
    static constexpr int N = _N;       // Order of the spline.
    static constexpr int DEG = _N - 1; // Degree of the spline.

    using MatN = Eigen::Matrix<double, _N, _N>;
    using VecN = Eigen::Matrix<double, _N, 1>;
    using Vec3 = Eigen::Vector3d;
    using Mat3 = Eigen::Matrix3d;

    const View* view;
    const Scene* scene;
    std::vector<int> track_ids;
    double u_so3;
    double inv_so3_dt;
    double u_r3;
    double inv_r3_dt;

    RSReprojectionCostFunctorSplit(const View* view, const Scene* scene,
                                   double u_so3, double u_r3, double inv_so3_dt,
                                   double inv_r3_dt,
                                   const std::vector<int>& track_ids)
        : view(view),
          scene(scene),
          u_so3(u_so3),
          u_r3(u_r3),
          inv_so3_dt(inv_so3_dt),
          inv_r3_dt(inv_r3_dt),
          track_ids(track_ids)
    {
    }

    template <class T>
    bool operator()(const T* const* sKnots, T* sResiduals) const
    {
        using Vector3 = Eigen::Vector3<T>;
        using Vector4 = Eigen::Vector4<T>;
        using Vector1 = Eigen::Matrix<T, 1, 1>;
        using Matrix4 = Eigen::Matrix4<T>;

        constexpr int N2 = 2 * N;
        Eigen::Map<const Sophus::SE3<T>> const T_i_c(sKnots[N2]);
        Eigen::Map<const Vector1> const line_delay(sKnots[N2 + 1]);

        const auto cam = view->camera();
        const auto cam_model = cam.cameraIntrinsicsModel();

        T intr[10];
        for (int i = 0; i < cam.cameraIntrinsics()->numParameters(); ++i) {
            intr[i] = T(cam.intrinsics()[i]);
        }

        // if we have a rolling shutter cam we will always need to evaluate with
        // line delay
        for (size_t i = 0; i < track_ids.size(); ++i) {
            const auto feature = *view->featureOf(track_ids[i]);

            // get time for respective RS line
            const T y_coord = T(feature.y()) * line_delay[0];
            const T t_so3_row = T(u_so3) + y_coord;
            const T t_r3_row = T(u_r3) + y_coord;

            Sophus::SO3<T> R_w_i;
            CeresSplineHelper<T, N>::template evaluate_lie<Sophus::SO3>(
                sKnots, t_so3_row, T(inv_so3_dt), &R_w_i);

            Vector3 t_w_i;
            CeresSplineHelper<T, N>::template evaluate<3, 0>(
                sKnots + N, t_r3_row, T(inv_r3_dt), &t_w_i);

            Sophus::SE3<T> T_w_c = Sophus::SE3<T>(R_w_i, t_w_i) * T_i_c;
            Matrix4 T_c_w_matrix = T_w_c.inverse().matrix();

            // get corresponding 3d point
            Eigen::Map<const Vector4> const scene_point(sKnots[N2 + 2 + i]);

            Vector3 p3d = (T_c_w_matrix * scene_point).hnormalized();

            T reprojection[2];
            bool success = false;
            switch (cam_model) {
                case CameraIntrinsicsType::DivisionUndistortion:
                    success = DivisionUndistortionCameraModel::spaceToPixel(
                        intr, p3d.data(), reprojection);
                    break;
                case CameraIntrinsicsType::DoubleSphere:
                    success = DoubleSphereCameraModel::spaceToPixel(
                        intr, p3d.data(), reprojection);
                    break;
                case CameraIntrinsicsType::Pinhole:
                    success = PinholeCameraModel::spaceToPixel(intr, p3d.data(),
                                                               reprojection);
                    break;
                case CameraIntrinsicsType::PinholeRadialTangential:
                    success = PinholeRadialTangentialCameraModel::spaceToPixel(
                        intr, p3d.data(), reprojection);
                    break;
                case CameraIntrinsicsType::Fisheye:
                    success = FisheyeCameraModel::spaceToPixel(intr, p3d.data(),
                                                               reprojection);
                    break;
                case CameraIntrinsicsType::ExtendedUnified:
                    success = ExtendedUnifiedCameraModel::spaceToPixel(
                        intr, p3d.data(), reprojection);
                    break;
                case CameraIntrinsicsType::Omnidirectional:
                    success = OmnidirectionalCameraModel::spaceToPixel(
                        intr, p3d.data(), reprojection);
                    break;
                case CameraIntrinsicsType::Orthographic:
                    success = OrthographicCameraModel::spaceToPixel(
                        intr, p3d.data(), reprojection);
                    break;
                default:
                    break;
            }

            if (!success) {
                sResiduals[2 * i + 0] = T(1e10);
                sResiduals[2 * i + 1] = T(1e10);
            }
            else {
                const T inv_info_x =
                    T(1. / ceres::sqrt(feature.covariance(0, 0)));
                const T inv_info_y =
                    T(1. / ceres::sqrt(feature.covariance(1, 1)));
                sResiduals[2 * i + 0] =
                    inv_info_x * (reprojection[0] - T(feature.x()));
                sResiduals[2 * i + 1] =
                    inv_info_y * (reprojection[1] - T(feature.y()));
            }
        }

        return true;
    }
};

// template <int _N>
// struct RSInvDepthReprojCostFunctorSplit : public CeresSplineHelper<double,
// _N> {
//  static constexpr int N = _N;       // Order of the spline.
//  static constexpr int DEG = _N - 1; // Degree of the spline.

//  using MatN = Eigen::Matrix<double, _N, _N>;
//  using VecN = Eigen::Matrix<double, _N, 1>;

//  using Vec3 = Eigen::Vector3d;
//  using Mat3 = Eigen::Matrix3d;

//  RSInvDepthReprojCostFunctorSplit(
//      const View *view, const Reconstruction *image_data,
//      const Sophus::SE3d &T_i_c, const TrackId &track_id,
//      const double u_so3_obs, const double u_r3_obs, const double u_so3_ref,
//      const double u_r3_ref, const double inv_so3_dt, const double inv_r3_dt,
//      std::vector<int> &ptr_offsets, const double weight = 1.0)
//      : view(view), image_data(image_data), T_i_c(T_i_c),
//        T_c_i(T_i_c.inverse()), track_id(track_id), u_so3_obs(u_so3_obs),
//        u_r3_obs(u_r3_obs), u_so3_ref(u_so3_ref), u_r3_ref(u_r3_ref),
//        inv_so3_dt(inv_so3_dt), inv_r3_dt(inv_r3_dt),
//        ptr_offsets(ptr_offsets), weight(weight) {}

//  template <class T>
//  bool operator()(T const *const *sKnots, T *sResiduals) const {
//    using Vector2 = Eigen::Vector2<T>;
//    using Vector3 = Eigen::Vector3<T>;
//    using Vector4 = Eigen::Vector4<T>;
//    using Vector1 = Eigen::Matrix<T, 1, 1>;

//    using Matrix4 = Eigen::Matrix4<T>;

//    const auto cam = view->Camera();
//    const auto cam_model = cam.GetCameraIntrinsicsModelType();

//    T intr[10];
//    for (int i = 0; i < cam.CameraIntrinsics()->NumParameters(); ++i) {
//      intr[i] = T(cam.intrinsics()[i]);
//    }

//    // evaluate spline pose for current observation of landmark
//    // const T y_coord = T(feature[1]) * line_delay[0];
//    const T t_so3_obs_row = T(u_so3_obs); // + y_coord;
//    const T t_r3_obs_row = T(u_r3_obs);   // + y_coord;
//    Sophus::SO3<T> R_obs_w_i;
//    CeresSplineHelper<T, N>::template evaluate_lie<Sophus::SO3>(
//        sKnots + ptr_offsets[0], t_so3_obs_row, T(inv_so3_dt), &R_obs_w_i);
//    Vector3 t_obs_w_i;
//    CeresSplineHelper<T, N>::template evaluate<3, 0>(
//        sKnots + ptr_offsets[2], t_r3_obs_row, T(inv_r3_dt), &t_obs_w_i);

//    const Vector2 feature(T((*view->GetFeature(track_id)).x()),
//                          T((*view->GetFeature(track_id)).y()));

//    // evaluate spline pose for reference observation of landmark
//    const T t_ref_so3_row = T(u_so3_ref); // + y_coord;
//    const T t_ref_r3_row = T(u_r3_ref);   // + y_coord;
//    Sophus::SO3<T> R_ref_w_i;
//    CeresSplineHelper<T, N>::template evaluate_lie<Sophus::SO3>(
//        sKnots + ptr_offsets[1], t_ref_so3_row, T(inv_so3_dt), &R_ref_w_i);
//    Vector3 t_ref_w_i;
//    CeresSplineHelper<T, N>::template evaluate<3, 0>(
//        sKnots + ptr_offsets[3], t_ref_r3_row, T(inv_r3_dt), &t_ref_w_i);

//    // get inverse depth point
//    T inverse_depth = T(1.0) / *(sKnots + ptr_offsets[4])[0];

//    T reprojection[2];
//    const Eigen::Vector3d bearing_d =
//        image_data->Track(track_id)->ReferenceBearingVector();
//    Vector3 bearing;
//    bearing << T(bearing_d[0]), T(bearing_d[1]), T(bearing_d[2]);
//    bearing *= inverse_depth;

//    bool success = false;

//    // inverse depth projection
//    // 1. convert point from bearing vector to 3d point using
//    // inverse depth from reference view and transform from camera to IMU
//    // reference frame
//    Vector3 X_ref = T_i_c.so3() * bearing + T_i_c.translation();
//    // 2. Transform point from IMU to world frame
//    Vector3 X = R_ref_w_i * X_ref + t_ref_w_i;
//    // 3. Transform point from world to IMU reference frame at observation
//    view Vector3 X_obs = R_obs_w_i.inverse() * (X - t_obs_w_i);
//    // 4. Transform point from IMU reference frame to camera frame
//    Vector3 X_camera = T_c_i.so3() * X_obs + T_c_i.translation();

//    // project back to camera space
//    if (CameraIntrinsicsType::DIVISION_undistortION ==
//        cam.GetCameraIntrinsicsModelType()) {
//      success =
//          DivisionundistortionCameraModel::spaceToPixel(
//              intr, X_camera.data(), reprojection);
//    } else if (CameraIntrinsicsType::DOUBLE_SPHERE == cam_model) {
//      success = DoubleSphereCameraModel::spaceToPixel(
//          intr, X_camera.data(), reprojection);
//    } else if (CameraIntrinsicsType::PINHOLE == cam_model) {
//      success = PinholeCameraModel::spaceToPixel(
//          intr, X_camera.data(), reprojection);
//    } else if (CameraIntrinsicsType::FISHEYE == cam_model) {
//      success = FisheyeCameraModel::spaceToPixel(
//          intr, X_camera.data(), reprojection);
//    } else if (CameraIntrinsicsType::EXTENDED_UNIFIED ==
//               cam_model) {
//      success = ExtendedUnifiedCameraModel::spaceToPixel(
//          intr, X_camera.data(), reprojection);
//    }else if (CameraIntrinsicsType::PINHOLE_RADIAL_TANGENTIAL ==
//              cam_model) {
//     success =
//     PinholeRadialTangentialCameraModel::spaceToPixel(
//         intr, X_camera.data(), reprojection);
//   }
//    if (!success) {
//      sResiduals[0] = T(1e10);
//      sResiduals[1] = T(1e10);
//    } else {
//      sResiduals[0] = T(weight) * (reprojection[0] - T(feature[0]));
//      sResiduals[1] = T(weight) * (reprojection[1] - T(feature[1]));
//    }

//    return true;
//  }
//  const View *view;
//  const Reconstruction *image_data;
//  TrackId track_id;
//  Sophus::SE3d T_i_c;
//  Sophus::SE3d T_c_i;
//  // landmark OBServation and REFerence times
//  double u_so3_obs;
//  double u_so3_ref;
//  double inv_so3_dt;
//  double u_r3_obs;
//  double u_r3_ref;
//  double inv_r3_dt;
//  double weight;
//  std::vector<int> ptr_offsets;
//};

} // namespace tl
