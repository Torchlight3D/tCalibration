#pragma once

#include <thread>

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <glog/logging.h>
#include <sophus/ceres_manifold.hpp>

#include <tCore/EnumUtils>
#include <tMath/Eigen/Utils>

#include "splinefactors.h"

namespace tl {

inline constexpr double GRAVITY_MAGN = 9.81;
inline constexpr double NS_TO_S = 1e-9;
inline constexpr double S_TO_NS = 1e9;

enum class OptimizationFlags
{
    None = 0,

    WorldPoint = 1 << 0,
    TIC = 1 << 1,
    ImuBiases = 1 << 2,
    ImuIntrinsics = 1 << 3,
    Gravity = 1 << 4,
    LineDelay = 1 << 5,
    Spline = 1 << 6,
    AccBias = 1 << 7,
    GyrBias = 1 << 8
};
MAKE_FLAGS(OptimizationFlags)

template <int _Order>
class SplineTrajectoryEstimator
{
public:
    enum
    {
        Order = _Order,
        Degree = _Order - 1
    };

    SplineTrajectoryEstimator()
        : SplineTrajectoryEstimator(0.1 * S_TO_NS, 0.1 * S_TO_NS, 0)
    {
    }
    SplineTrajectoryEstimator(int64_t dt_so3, int64_t dt_r3, int64_t t0)
        : _dt_so3(dt_so3),
          _dt_r3(dt_r3),
          _t0(t0),
          _gravity(0., 0., GRAVITY_MAGN)
    {
        _dt_so3_inv = S_TO_NS / _dt_so3;
        _dt_r3_inv = S_TO_NS / _dt_r3;

        _accl_params << 0, 0, 0, 1, 1, 1;
        _gyro_params << 0, 0, 0, 0, 0, 0, 1, 1, 1;
    }

    int64_t endTime() const
    {
        return _t0 + (_so3_knots.size() - Order + 1) * _dt_so3 - 1;
    }

    int64_t startTime() const { return _t0; }

    void SetTimes(int64_t dt_so3, int64_t dt_r3, int64_t start, int64_t end)
    {
        _dt_so3 = dt_so3;
        _dt_r3 = dt_r3;
        _t0 = start;
        _t_end = end;

        const int64_t duration = _t_end - _t0;
        nr_knots_so3_ = duration / _dt_so3 + _Order;
        nr_knots_r3_ = duration / _dt_r3 + _Order;
        _dt_so3_inv = S_TO_NS / _dt_so3;
        _dt_r3_inv = S_TO_NS / _dt_r3;
    }

    void setScene(const Scene& scene)
    {
        _scene = scene;
        // calculate all reference bearings
        //  const auto track_ids = image_data_.TrackIds();
        //  for (auto t = 0; t < track_ids.size(); ++t) {
        //    theia::Track *mut_track = image_data_.MutableTrack(track_ids[t]);
        //    theia::ViewId ref_view_id = mut_track->ReferenceViewId();
        //    const theia::View *v = image_data_.View(ref_view_id);
        //    const Eigen::Vector2d feat =
        //    (*v->GetFeature(track_ids[t])).point_; Eigen::Vector3d bearing =
        //    v->Camera().PixelToNormalizedCoordinates(feat); Eigen::Vector3d
        //    adjusted_point =
        //        mut_track->Point().head<3>() -
        //        mut_track->Point()[3] * v->Camera().GetPosition();
        //    Eigen::Vector3d rotated_point =
        //        v->Camera().GetOrientationAsRotationMatrix() * adjusted_point;
        ////    if (std::abs(rotated_point[2]) < 1e-10)
        ////        *mut_track->MutableInverseDepth() = 1 / 1e-10;
        ////    else
        ////        *mut_track->MutableInverseDepth() = 1 / rotated_point[2];
        //   *mut_track->MutableInverseDepth() = 1 / 0.5;
        //    mut_track->SetReferenceBearingVector(bearing);
        //  }
    }

    bool addAccelerometerMeasurement(const Eigen::Vector3d& meas, int64_t time,
                                     double weight)
    {
        double u_r3, u_so3, u_bias;
        int64_t s_r3, s_so3, s_bias;
        if (!CalcR3Times(time, u_r3, s_r3)) {
            LOG(INFO)
                << "Wrong time adding r3 accelerometer measurements. time_ns: "
                << time << " u_r3: " << u_r3 << " s_r3:" << s_r3;
            return false;
        }
        if (!CalcSO3Times(time, u_so3, s_so3)) {
            LOG(INFO)
                << "Wrong time adding so3 accelerometer measurements. time_ns: "
                << time << " u_r3: " << u_r3 << " s_r3:" << s_r3;
            return false;
        }
        if (!CalcTimes(time, u_bias, s_bias, _dt_accl_bias,
                       _accl_bias_spline.size(), kBiasSplineOrder)) {
            LOG(INFO) << "Wrong time adding accelerometer bias measurements. "
                         "time_ns: "
                      << time << " u_r3: " << u_bias << " s_r3:" << s_bias;
            return false;
        }

        using Functor = AccelerationCostFunctorSplit<Order>;

        auto cost = new ceres::DynamicAutoDiffCostFunction<Functor>(
            new Functor(meas, u_r3, _dt_r3_inv, u_so3, _dt_so3_inv, weight,
                        u_bias, _dt_accl_bias_inv));

        std::vector<double*> params;
        // SO3 spline
        for (auto i{0}; i < Order; ++i) {
            cost->AddParameterBlock(4);
            const int t = s_so3 + i;
            params.emplace_back(_so3_knots[t].data());
        }

        // R3 spline
        for (auto i{0}; i < Order; ++i) {
            cost->AddParameterBlock(3);
            const int t = s_r3 + i;
            params.emplace_back(_r3_knots[t].data());
        }

        // Bias spline
        for (auto i{0}; i < kBiasSplineOrder; ++i) {
            cost->AddParameterBlock(3);
            const int t = s_bias + i;
            params.emplace_back(_accl_bias_spline[t].data());
        }

        // Gravity
        cost->AddParameterBlock(3);
        params.emplace_back(_gravity.data());

        // Accelerometer intrinsics
        cost->AddParameterBlock(6);
        params.emplace_back(_accl_params.data());

        // Residuals. Acc diff
        cost->SetNumResiduals(3);

        _problem.AddResidualBlock(cost, nullptr, params);

        return true;
    }

    bool addGyroscopeMeasurement(const Eigen::Vector3d& meas, int64_t time,
                                 double weight)
    {
        double u_so3;
        int64_t s_so3;
        if (!CalcSO3Times(time, u_so3, s_so3)) {
            LOG(INFO)
                << "Wrong time adding so3 gyroscope measurements. time_ns: "
                << time << " u_r3: " << u_so3 << " s_r3:" << s_so3;
            return false;
        }

        double u_bias;
        int64_t s_bias;
        if (!CalcTimes(time, u_bias, s_bias, _dt_gyro_bias,
                       _gyro_bias_spline.size(), kBiasSplineOrder)) {
            LOG(INFO) << "Wrong time adding so3 gyroscope bias measurements. "
                         "time_ns: "
                      << time << " u_r3: " << u_bias << " s_r3:" << s_bias;
            return false;
        }

        using Functor = GyroCostFunctorSplit<Order, Sophus::SO3>;

        auto cost = new ceres::DynamicAutoDiffCostFunction<Functor>(new Functor(
            meas, u_so3, _dt_so3_inv, weight, u_bias, _dt_gyro_bias_inv));

        std::vector<double*> params;
        // SO3 spline
        for (auto i{0}; i < Order; i++) {
            cost->AddParameterBlock(4);
            const int t = s_so3 + i;
            params.emplace_back(_so3_knots[t].data());
        }

        // Bias spline
        for (int i = 0; i < kBiasSplineOrder; ++i) {
            cost->AddParameterBlock(3);
            const int t = s_bias + i;
            params.emplace_back(_gyro_bias_spline[t].data());
        }

        // Gyroscope intrinsics
        cost->AddParameterBlock(9);
        params.emplace_back(_gyro_params.data());

        // Residuals. Gyro diff
        cost->SetNumResiduals(3);

        _problem.AddResidualBlock(cost, nullptr, params);

        return true;
    }

    bool addGlobalShutterCameraMeasurement(const View* const view,
                                           double robust_loss_width)
    {
        const int64_t time = view->timestamp() * S_TO_NS;

        double u_r3 = 0.;
        int64_t s_r3 = 0;
        if (!CalcR3Times(time, u_r3, s_r3)) {
            LOG(INFO)
                << "Wrong time observation r3 vision measurements. time_ns: "
                << time << " u_r3: " << u_r3 << " s_r3:" << s_r3;
            return false;
        }
        double u_so3 = 0.;
        int64_t s_so3 = 0;
        if (!CalcSO3Times(time, u_so3, s_so3)) {
            LOG(INFO)
                << "Wrong time reference so3 vision measurements. time_ns: "
                << time << " u_r3: " << u_so3 << " s_r3:" << s_so3;
            return false;
        }

        const auto trackIds = view->trackIds();

        using FunctorT = GSReprojectionCostFunctorSplit<Order>;

        auto cost =
            new ceres::DynamicAutoDiffCostFunction<FunctorT>(new FunctorT(
                view, &_scene, u_so3, u_r3, _dt_so3_inv, _dt_r3_inv, trackIds));

        std::vector<double*> params;
        // SO3 spline
        for (int i = 0; i < Order; i++) {
            cost->AddParameterBlock(4);
            const int t = s_so3 + i;
            params.emplace_back(_so3_knots[t].data());
        }

        // R3 spline
        for (int i = 0; i < Order; i++) {
            cost->AddParameterBlock(3);
            const int t = s_r3 + i;
            params.emplace_back(_r3_knots[t].data());
        }

        // Camera to IMU transformation
        cost->AddParameterBlock(7);
        params.emplace_back(_T_ic.data());

        // Object points
        for (size_t i = 0; i < trackIds.size(); ++i) {
            cost->AddParameterBlock(4);
            params.emplace_back(_scene.rTrack(trackIds[i])->rPosition().data());
            tracks_in_problem_.insert(trackIds[i]);
        }

        // Residuals. Pixel diffs
        cost->SetNumResiduals(trackIds.size() * 2);

        auto loss = new ceres::HuberLoss(robust_loss_width);

        _problem.AddResidualBlock(cost, loss, params);

        return true;
    }

    bool addRollingShutterCameraMeasurement(const View* const view,
                                            double robust_loss_width = 0.0)
    {
        const int64_t time = view->timestamp() * S_TO_NS;

        double u_r3 = 0.0;
        int64_t s_r3 = 0;
        if (!CalcR3Times(time, u_r3, s_r3)) {
            LOG(INFO)
                << "Wrong time observation r3 vision measurements. time_ns: "
                << time << " u_r3: " << u_r3 << " s_r3:" << s_r3;
            return false;
        }

        double u_so3 = 0.0;
        int64_t s_so3 = 0;
        if (!CalcSO3Times(time, u_so3, s_so3)) {
            LOG(INFO)
                << "Wrong time reference so3 vision measurements. time_ns: "
                << time << " u_r3: " << u_so3 << " s_r3:" << s_so3;
            return false;
        }

        const auto trackIds = view->trackIds();

        using Functor = RSReprojectionCostFunctorSplit<Order>;

        auto cost = new ceres::DynamicAutoDiffCostFunction<Functor>(new Functor(
            view, &_scene, u_so3, u_r3, _dt_so3_inv, _dt_r3_inv, trackIds));

        std::vector<double*> params;
        // SO3 spline
        for (int i = 0; i < Order; i++) {
            cost->AddParameterBlock(4);
            const int t = s_so3 + i;
            params.emplace_back(_so3_knots[t].data());
        }

        // R3 spline
        for (int i = 0; i < Order; i++) {
            cost->AddParameterBlock(3);
            const int t = s_r3 + i;
            params.emplace_back(_r3_knots[t].data());
        }

        // Camera to IMU transformation
        cost->AddParameterBlock(7);
        params.emplace_back(_T_ic.data());

        // Line delay for rolling shutter cameras
        cost->AddParameterBlock(1);
        params.emplace_back(&_lineDelay);

        // Object points
        for (size_t i = 0; i < trackIds.size(); ++i) {
            cost->AddParameterBlock(4);
            params.emplace_back(_scene.rTrack(trackIds[i])->rPosition().data());
            tracks_in_problem_.insert(trackIds[i]);
        }

        // Residuals. Pixel diffs.
        cost->SetNumResiduals(trackIds.size() * 2);

        if (robust_loss_width == 0.0) {
            _problem.AddResidualBlock(cost, nullptr, params);
        }
        else {
            auto loss = new ceres::HuberLoss(robust_loss_width);
            _problem.AddResidualBlock(cost, loss, params);
        }

        return true;
    }

    void initBiasSplines(const Eigen::Vector3d& accl_init_bias,
                         const Eigen::Vector3d& gyr_init_bias,
                         int64_t dt_accl_bias = 500000000,
                         int64_t dt_gyro_bias = 500000000,
                         double max_accl_range = 1.0,
                         double max_gyro_range = 1e-2)
    {
        _accl_bias_range = max_accl_range;
        _gyro_bias_range = max_gyro_range;

        _dt_accl_bias = dt_accl_bias;
        _dt_gyro_bias = dt_gyro_bias;
        _dt_accl_bias_inv = 1. / _dt_accl_bias;
        _dt_gyro_bias_inv = 1. / _dt_gyro_bias;

        const auto duration = _t_end - _t0;
        const auto nr_knots_accl_bias_ =
            duration / _dt_accl_bias + kBiasSplineOrder;
        const auto nr_knots_gyro_bias_ =
            duration / _dt_gyro_bias + kBiasSplineOrder;

        LOG(INFO) << "Initializing " << nr_knots_accl_bias_
                  << " acceleration bias knots with: "
                  << accl_init_bias.transpose() << " m2/s";
        LOG(INFO) << "Initializing " << nr_knots_gyro_bias_
                  << " gyroscope bias knots. " << gyr_init_bias.transpose()
                  << " rad/s";

        _accl_bias_spline.resize(nr_knots_accl_bias_, accl_init_bias);
        _gyro_bias_spline.resize(nr_knots_gyro_bias_, gyr_init_bias);
    }

    void initCameraPoses()
    {
        // Clear data
        _so3_knots = std::vector<Sophus::SO3d>(nr_knots_so3_);
        _r3_knots = std::vector<Eigen::Vector3d>(nr_knots_r3_);

        // 1. Interpolate visual poses
        std::map<double, Eigen::Quaterniond> quat_vis_map;
        std::map<double, Eigen::Vector3d> translations_map;

        // Sort data
        for (const auto& viewId : _scene.viewIds()) {
            const auto* view = _scene.view(viewId);
            const double t_s = view->timestamp();
            const Eigen::Quaterniond q_wc{
                view->camera().orientationAsRotationMatrix().transpose()};
            const Sophus::SE3d T_wc{q_wc, view->camera().position()};
            const Sophus::SE3d T_wi = T_wc * _T_ic.inverse();

            quat_vis_map[t_s] = T_wi.so3().unit_quaternion();
            translations_map[t_s] = T_wi.translation();
        }

        std::vector<double> t_vis;
        std::vector<Eigen::Quaterniond> quat_vis;
        std::vector<Eigen::Vector3d> translations;
        for (const auto& [t_s, q] : quat_vis_map) {
            quat_vis.push_back(q);
            t_vis.push_back(t_s);
        }
        for (auto const& [_, t] : translations_map) {
            translations.push_back(t);
        }

        // 2. Get timestamp where we want to interpolate
        std::vector<double> t_so3_spline;
        for (int i = 0; i < nr_knots_so3_; ++i) {
            const double t = i * _dt_so3 * NS_TO_S;
            t_so3_spline.push_back(t);
        }

        std::vector<double> t_r3_spline;
        for (int i = 0; i < nr_knots_r3_; ++i) {
            const double t = i * _dt_r3 * NS_TO_S;
            t_r3_spline.push_back(t);
        }

        // 3. Interpolate
        std::vector<Eigen::Quaterniond> interp_spline_quats;
        std::vector<Eigen::Vector3d> interpo_spline_trans;
        math::InterpolateQuaternions(t_vis, t_so3_spline, quat_vis,
                                     interp_spline_quats);
        math::InterpolateVector3s(t_vis, t_r3_spline, translations,
                                  interpo_spline_trans);

        // 4. Save to knots
        for (int i = 0; i < nr_knots_so3_; ++i) {
            _so3_knots[i] = Sophus::SO3d(interp_spline_quats[i]);
        }
        for (int i = 0; i < nr_knots_r3_; ++i) {
            _r3_knots[i] = interpo_spline_trans[i];
        }
    }

    ceres::Solver::Summary optimize(int maxIterations, OptimizationFlags flags)
    {
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.max_num_iterations = maxIterations;
        options.num_threads = std::thread::hardware_concurrency();
        options.minimizer_progress_to_stdout = true;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.function_tolerance = 1e-4;
        options.parameter_tolerance = 1e-7;
        options.preconditioner_type = ceres::CLUSTER_TRIDIAGONAL;
        options.use_inner_iterations = true;

        setConstantParameters(flags);

        ceres::Solver::Summary summary;
        ceres::Solve(options, &_problem, &summary);

        LOG(INFO) << summary.FullReport();

        return summary;
    }

    Eigen::Vector3d gravity() const { return _gravity; }
    void setGravity(const Eigen::Vector3d& g) { _gravity = g; }

    Sophus::SE3d cameraToImuTransform() const { return _T_ic; }
    void setCameraToImuTransform(const Sophus::SE3d& T) { _T_ic = T; }

    double cameraLineDelay() const { return _lineDelay; }
    void setCameraLineDelay(double delay) { _lineDelay = delay; }

    void setImuIntrinsics(const ImuIntrinsics& accl_intrinsics,
                          const ImuIntrinsics& gyro_intrinsics)
    {
        _accl_params << accl_intrinsics.misYZ(), accl_intrinsics.misZY(),
            accl_intrinsics.misZX(), accl_intrinsics.scaleX(),
            accl_intrinsics.scaleY(), accl_intrinsics.scaleZ();

        _gyro_params << gyro_intrinsics.misYZ(), gyro_intrinsics.misZY(),
            gyro_intrinsics.misZX(), gyro_intrinsics.misXZ(),
            gyro_intrinsics.misXY(), gyro_intrinsics.misYX(),
            gyro_intrinsics.scaleX(), gyro_intrinsics.scaleY(),
            gyro_intrinsics.scaleZ();
    }

    ImuIntrinsics accelerometerlIntrinsics(int64_t time) const
    {
        const auto bias = accelerometerBias(time);
        ImuIntrinsics intrinsics(_accl_params[0], _accl_params[1],
                                 _accl_params[2], 0, 0, 0, _accl_params[3],
                                 _accl_params[4], _accl_params[5], bias[0],
                                 bias[1], bias[2]);
        return intrinsics;
    }

    ImuIntrinsics gyroscopeIntrinsics(int64_t time) const
    {
        const auto bias = gyroscopeBias(time);
        ImuIntrinsics intrinsics(
            _gyro_params[0], _gyro_params[1], _gyro_params[2], _gyro_params[3],
            _gyro_params[4], _gyro_params[5], _gyro_params[6], _gyro_params[7],
            _gyro_params[8], bias[0], bias[1], bias[2]);
        return intrinsics;
    }

    Sophus::SE3d knot(int i) const { return {_so3_knots[i], _r3_knots[i]}; }

    bool pose(int64_t time, Sophus::SE3d& pose) const
    {
        double u_r3;
        int64_t s_r3;
        if (!CalcR3Times(time, u_r3, s_r3)) {
            return false;
        }

        double u_so3;
        int64_t s_so3;
        if (!CalcSO3Times(time, u_so3, s_so3)) {
            return false;
        }

        Sophus::SO3d rot;
        {
            std::vector<const double*> vec;
            for (int i = 0; i < Order; ++i) {
                vec.emplace_back(_so3_knots[s_so3 + i].data());
            }

            SplineBase<Order>::template evaluate_lie<double, Sophus::SO3>(
                &vec[0], u_so3, _dt_so3_inv, &rot);
        }

        Eigen::Vector3d trans;
        {
            std::vector<const double*> vec;
            for (int i = 0; i < Order; ++i) {
                vec.emplace_back(_r3_knots[s_r3 + i].data());
            }

            SplineBase<Order>::template evaluate<double, 3, 0>(
                &vec[0], u_r3, _dt_r3_inv, &trans);
        }

        pose = Sophus::SE3d(rot, trans);

        return true;
    }

    bool position(int64_t time, Eigen::Vector3d& position)
    {
        double u_r3;
        int64_t s_r3;
        if (!CalcR3Times(time, u_r3, s_r3)) {
            return false;
        }

        std::vector<const double*> vec;
        for (int i = 0; i < Order; ++i) {
            vec.emplace_back(_r3_knots[s_r3 + i].data());
        }

        SplineBase<Order>::template evaluate<double, 3, 0>(
            &vec[0], u_r3, _dt_r3_inv, &position);

        return true;
    }

    bool angularVelocity(int64_t time, Eigen::Vector3d& omega) const
    {
        double u_so3;
        int64_t s_so3;
        if (!CalcSO3Times(time, u_so3, s_so3)) {
            return false;
        }

        std::vector<const double*> vec;
        for (int i = 0; i < Order; ++i) {
            vec.emplace_back(_so3_knots[s_so3 + i].data());
        }

        SplineBase<Order>::template evaluate_lie<double, Sophus::SO3>(
            &vec[0], u_so3, _dt_so3_inv, nullptr, &omega);

        return true;
    }

    bool acceleration(int64_t time, Eigen::Vector3d& acceleration) const
    {
        double u_r3, u_so3;
        int64_t s_r3, s_so3;
        if (!CalcR3Times(time, u_r3, s_r3)) {
            return false;
        }
        if (!CalcSO3Times(time, u_so3, s_so3)) {
            return false;
        }

        Sophus::SO3d rot;
        {
            std::vector<const double*> vec;
            for (int i = 0; i < Order; ++i) {
                vec.emplace_back(_so3_knots[s_so3 + i].data());
            }

            SplineBase<Order>::template evaluate_lie<double, Sophus::SO3>(
                &vec[0], u_so3, _dt_so3_inv, &rot);
        }

        Eigen::Vector3d trans_accel_world;
        {
            std::vector<const double*> vec;
            for (int i = 0; i < Order; ++i) {
                vec.emplace_back(_r3_knots[s_r3 + i].data());
            }

            SplineBase<Order>::template evaluate<double, 3, 2>(
                &vec[0], u_r3, _dt_r3_inv, &trans_accel_world);
        }

        acceleration = rot.inverse() * (trans_accel_world + _gravity);

        return true;
    }

    size_t GetNumSO3Knots() const { return _so3_knots.size(); }

    size_t GetNumR3Knots() const { return _r3_knots.size(); }

    Eigen::Vector3d gyroscopeBias(int64_t time) const
    {
        Eigen::Vector3d bias;
        bias.setZero();

        double u;
        int64_t s;
        if (!CalcTimes(time, u, s, _dt_gyro_bias, _gyro_bias_spline.size(),
                       kBiasSplineOrder)) {
            return bias;
        }

        std::vector<const double*> vec;
        for (int i = 0; i < kBiasSplineOrder; ++i) {
            vec.emplace_back(_gyro_bias_spline[s + i].data());
        }

        SplineBase<kBiasSplineOrder>::evaluate<double, 3, 0>(
            &vec[0], u, _dt_gyro_bias_inv, &bias);

        return bias;
    }

    Eigen::Vector3d accelerometerBias(int64_t time) const
    {
        Eigen::Vector3d bias;
        bias.setZero();

        double u;
        int64_t s;
        if (!CalcTimes(time, u, s, _dt_accl_bias, _accl_bias_spline.size(),
                       kBiasSplineOrder)) {
            return bias;
        }

        std::vector<const double*> vec;
        for (int i = 0; i < kBiasSplineOrder; ++i) {
            vec.emplace_back(_accl_bias_spline[s + i].data());
        }

        SplineBase<kBiasSplineOrder>::evaluate<double, 3, 0>(
            &vec[0], u, _dt_accl_bias_inv, &bias);

        return bias;
    }

    double GetMeanReprojectionError()
    {
        // ConvertInvDepthPointsToHom();

        auto totalError{0.};
        auto numPoints{0};
        for (const auto& viewId : _scene.viewIds()) {
            const auto* view = _scene.view(viewId);
            const auto trackIds = view->trackIds();

            if (trackIds.empty()) {
                continue;
            }

            const auto numTracks = trackIds.size();
            const int64_t time = view->timestamp() * S_TO_NS;

            double u_r3;
            int64_t s_r3;
            if (!CalcR3Times(time, u_r3, s_r3)) {
                return 0.0;
            }
            double u_so3;
            int64_t s_so3;
            if (!CalcSO3Times(time, u_so3, s_so3)) {
                return 0.0;
            }

            using FunctorT = RSReprojectionCostFunctorSplit<Order>;

            auto cost = new ceres::DynamicAutoDiffCostFunction<FunctorT>(
                new FunctorT(view, &_scene, u_so3, u_r3, _dt_so3_inv,
                             _dt_r3_inv, trackIds));

            std::vector<double*> params;
            // SO3 spline
            for (int i = 0; i < Order; i++) {
                cost->AddParameterBlock(4);
                const int t = s_so3 + i;
                params.emplace_back(_so3_knots[t].data());
            }

            // R3 spline
            for (int i = 0; i < Order; i++) {
                cost->AddParameterBlock(3);
                const int t = s_r3 + i;
                params.emplace_back(_r3_knots[t].data());
            }

            // Camera to IMU transformation
            cost->AddParameterBlock(7);
            params.emplace_back(_T_ic.data());

            // Line delay for rolling shutter cameras
            cost->AddParameterBlock(1);
            params.emplace_back(&_lineDelay);

            // Object points
            for (size_t i{0}; i < numTracks; ++i) {
                cost->AddParameterBlock(4);
                params.emplace_back(
                    _scene.rTrack(trackIds[i])->rPosition().data());
            }

            cost->SetNumResiduals(2 * numTracks);

            {
                Eigen::VectorXd residuals;
                residuals.setZero(numTracks * 2);

                cost->Evaluate(&params[0], residuals.data(), nullptr);

                for (size_t i{0}; i < numTracks; i++) {
                    const Eigen::Vector2d offset = residuals.segment<2>(2 * i);
                    totalError += offset.norm();
                    numPoints++;
                }
            }
        }

        const auto rpe = totalError / numPoints;

        LOG(INFO) << "Mean reprojection error " << rpe
                  << " number residuals: " << numPoints;

        return rpe;
    }

    void toScene(Scene* scene) const
    {
        for (const auto& viewId : _scene.viewIds()) {
            const int64_t time = _scene.view(viewId)->timestamp() * S_TO_NS;

            Sophus::SE3d T_wi;
            pose(time, T_wi);

            const Sophus::SE3d T_wc = T_wi * _T_ic;

            const auto newViewId =
                scene->addView(std::to_string(time), time, CameraId{0});
            auto newView = scene->rView(newViewId);
            newView->setEstimated(true);
            auto& camera = newView->rCamera();
            camera.setOrientationFromRotationMatrix(
                T_wc.rotationMatrix().transpose());
            camera.setPosition(T_wc.translation());
        }

        // ConvertInvDepthPointsToHom();

        for (const auto& trackId : _scene.trackIds()) {
            const auto newTrackId = scene->addTrack();
            auto newTrack = scene->rTrack(newTrackId);
            newTrack->rPosition() = _scene.track(trackId)->position();
            newTrack->setEstimated(true);
        }
    }

    void ConvertInvDepthPointsToHom()
    {
        for (const auto& trackId : _scene.trackIds()) {
            auto track = _scene.rTrack(trackId);
            const auto refView = _scene.view(track->referenceViewId());
            const auto inverseDepth = *track->inverseDepth();

            const Eigen::Vector3d bearing =
                refView->camera().pixelToUnitDepthRay(
                    refView->featureOf(trackId)->pos);

            const int64_t time = refView->timestamp() * S_TO_NS;

            Sophus::SE3d T_wi;
            pose(time, T_wi);

            Eigen::Vector3d X_ref =
                _T_ic.so3() * (bearing - inverseDepth * _T_ic.translation());

            // Transform point from IMU to world frame
            Eigen::Vector3d X =
                T_wi.so3() * X_ref + T_wi.translation() * inverseDepth;
            track->rPosition() = X.homogeneous();
            track->setEstimated(true);
        }
    }

private:
    bool CalcSO3Times(int64_t time, double& u_so3, int64_t& s_so3) const
    {
        return CalcTimes(time, u_so3, s_so3, _dt_so3, _so3_knots.size());
    }

    bool CalcR3Times(int64_t time, double& u_r3, int64_t& s_r3) const
    {
        return CalcTimes(time, u_r3, s_r3, _dt_r3, _r3_knots.size());
    }

    bool CalcTimes(int64_t time, double& u, int64_t& s, int64_t dt,
                   size_t nr_knots, int N = Order) const
    {
        const int64_t st_ns = (time - _t0);

        if (st_ns < 0.0) {
            u = 0.0;
            return false;
        }

        s = st_ns / dt;
        if (s < 0) {
            return false;
        }

        if (size_t(s + N) > nr_knots) {
            return false;
        }

        u = double(st_ns % dt) / double(dt);
        return true;
    }

    void setConstantParameters(OptimizationFlags flags)
    {
        // IMU to camera transform
        if (_problem.HasParameterBlock(_T_ic.data())) {
            if ((flags & OptimizationFlags::TIC) == OptimizationFlags::None) {
                _problem.SetParameterBlockConstant(_T_ic.data());
                LOG(INFO) << "Keeping T_I_C constant.";
            }
            else {
                _problem.SetManifold(_T_ic.data(),
                                     new Sophus::Manifold<Sophus::SE3>());
                _problem.SetParameterBlockVariable(_T_ic.data());
                LOG(INFO) << "Optimizing T_I_C.";
            }
        }

        // Camera line delay (for rolling shutter camera)
        if (_problem.HasParameterBlock(&_lineDelay) && _lineDelay != 0.0) {
            if ((flags & OptimizationFlags::LineDelay) ==
                OptimizationFlags::None) {
                _problem.SetParameterBlockConstant(&_lineDelay);
                LOG(INFO) << "Keeping camera line delay constant at: "
                          << _lineDelay;
            }
            else {
                _problem.SetParameterBlockVariable(&_lineDelay);
                LOG(INFO) << "Optimizing camera line delay.";
            }
        }

        // Gravity
        if (_problem.HasParameterBlock(_gravity.data())) {
            if ((flags & OptimizationFlags::Gravity) ==
                OptimizationFlags::None) {
                LOG(INFO) << "Keeping gravity direction constant at: "
                          << _gravity.transpose();

                _problem.SetParameterBlockConstant(_gravity.data());
            }
            else {
                if (_problem.HasParameterBlock(_gravity.data()))
                    _problem.SetParameterBlockVariable(_gravity.data());
                LOG(INFO) << "Optimizing gravity direction.";
            }
        }

        // World points
        if ((flags & OptimizationFlags::WorldPoint) ==
            OptimizationFlags::None) {
            LOG(INFO) << "Keeping object points constant.";
            for (const auto& tid : tracks_in_problem_) {
                const auto track = _scene.rTrack(tid)->rPosition().data();
                if (_problem.HasParameterBlock(track))
                    _problem.SetParameterBlockConstant(track);
            }
        }
        else {
            for (const auto& tid : tracks_in_problem_) {
                const auto track = _scene.rTrack(tid)->rPosition().data();
                if (_problem.HasParameterBlock(track)) {
                    _problem.SetParameterBlockVariable(track);
                    _problem.SetManifold(track, new ceres::SphereManifold<4>());
                }
            }
            LOG(INFO) << "Optimizing object points.";
        }

        // IMU intrinsics
        if (_problem.HasParameterBlock(_accl_params.data()) &&
            _problem.HasParameterBlock(_gyro_params.data())) {
            if ((flags & OptimizationFlags::ImuIntrinsics) ==
                OptimizationFlags::None) {
                LOG(INFO) << "Keeping IMU intrinsics constant.";
                _problem.SetParameterBlockConstant(_accl_params.data());
                _problem.SetParameterBlockConstant(_gyro_params.data());
            }
            else {
                _problem.SetParameterBlockVariable(_accl_params.data());
                _problem.SetParameterBlockVariable(_gyro_params.data());
                LOG(INFO) << "Optimizing IMU intrinsics.";
            }
        }

        // Orientation (SO3) spline
        for (size_t i = 0; i < _so3_knots.size(); ++i) {
            if (_problem.HasParameterBlock(_so3_knots[i].data())) {
                _problem.SetManifold(_so3_knots[i].data(),
                                     new Sophus::Manifold<Sophus::SO3>());
            }
        }
        if ((flags & OptimizationFlags::Spline) == OptimizationFlags::None) {
            // set knots constant if asked
            for (size_t i = 0; i < _r3_knots.size(); ++i) {
                if (_problem.HasParameterBlock(_r3_knots[i].data())) {
                    _problem.SetParameterBlockConstant(_r3_knots[i].data());
                }
            }
            for (size_t i = 0; i < _so3_knots.size(); ++i) {
                if (_problem.HasParameterBlock(_so3_knots[i].data())) {
                    _problem.SetParameterBlockConstant(_so3_knots[i].data());
                }
            }
        }
        else {
            // set knots constant if asked
            for (size_t i = 0; i < _r3_knots.size(); ++i) {
                if (_problem.HasParameterBlock(_r3_knots[i].data())) {
                    _problem.SetParameterBlockVariable(_r3_knots[i].data());
                }
            }
            for (size_t i = 0; i < _so3_knots.size(); ++i) {
                if (_problem.HasParameterBlock(_so3_knots[i].data())) {
                    _problem.SetParameterBlockVariable(_so3_knots[i].data());
                }
            }
        }

        // Acceleration bias
        if ((flags & OptimizationFlags::AccBias) == OptimizationFlags::None ||
            (flags & OptimizationFlags::ImuBiases) == OptimizationFlags::None) {
            LOG(INFO) << "Optimizing accelerometer bias spline.";
            for (int i = 0; i < _accl_bias_spline.size(); ++i) {
                if (_problem.HasParameterBlock(_accl_bias_spline[i].data())) {
                    for (int d = 0; d < 3; ++d) {
                        _problem.SetParameterLowerBound(
                            _accl_bias_spline[i].data(), d, -_accl_bias_range);
                        _problem.SetParameterUpperBound(
                            _accl_bias_spline[i].data(), d, _accl_bias_range);
                    }
                    _problem.SetParameterBlockVariable(
                        _accl_bias_spline[i].data());
                }
            }
        }
        else {
            LOG(INFO) << "Fixing accelerometer bias spline.";
            for (int i = 0; i < _accl_bias_spline.size(); ++i) {
                if (_problem.HasParameterBlock(_accl_bias_spline[i].data())) {
                    _problem.SetParameterBlockConstant(
                        _accl_bias_spline[i].data());
                }
            }
        }

        // Gyroscope bias
        if ((flags & OptimizationFlags::GyrBias) == OptimizationFlags::None ||
            (flags & OptimizationFlags::ImuBiases) == OptimizationFlags::None) {
            LOG(INFO) << "Optimizing gyroscope bias spline.";
            for (int i = 0; i < _gyro_bias_spline.size(); ++i) {
                if (_problem.HasParameterBlock(_gyro_bias_spline[i].data())) {
                    for (int d = 0; d < 3; ++d) {
                        _problem.SetParameterLowerBound(
                            _gyro_bias_spline[i].data(), d, -_gyro_bias_range);
                        _problem.SetParameterUpperBound(
                            _gyro_bias_spline[i].data(), d, _gyro_bias_range);
                    }
                    _problem.SetParameterBlockVariable(
                        _gyro_bias_spline[i].data());
                }
            }
        }
        else {
            LOG(INFO) << "Fixing gyroscope bias spline.";
            for (int i = 0; i < _gyro_bias_spline.size(); ++i) {
                if (_problem.HasParameterBlock(_gyro_bias_spline[i].data())) {
                    _problem.SetParameterBlockConstant(
                        _gyro_bias_spline[i].data());
                }
            }
        }
    }

private:
    int64_t _t0;
    int64_t _t_end;

    // IMU SO3 and R3 spline
    std::vector<Sophus::SO3d> _so3_knots;
    std::vector<Eigen::Vector3d> _r3_knots;
    int64_t _dt_so3;
    int64_t _dt_r3;
    double _dt_so3_inv;
    double _dt_r3_inv;
    size_t nr_knots_so3_;
    size_t nr_knots_r3_;

    // Accelerometer and gyroscope bias spline
    std::vector<Eigen::Vector3d> _gyro_bias_spline;
    std::vector<Eigen::Vector3d> _accl_bias_spline;
    int64_t _dt_accl_bias;
    int64_t _dt_gyro_bias;
    double _dt_accl_bias_inv;
    double _dt_gyro_bias_inv;
    double _accl_bias_range = 1.0;
    double _gyro_bias_range = 1e-2;

    // Instances can be optimized
    Eigen::Matrix<double, 6, 1> _accl_params;
    Eigen::Matrix<double, 9, 1> _gyro_params;
    Sophus::SE3d _T_ic;
    Eigen::Vector3d _gravity;
    double _lineDelay = 0.;

    Scene _scene;

    std::set<TrackId> tracks_in_problem_;

    ceres::Problem _problem;
};

} // namespace tl
