#pragma once

#include <thread>

#include <sophus/so3.hpp>
#include <sophus/ceres_manifold.hpp>
#include <ceres/sphere_manifold.h>

#include <AxCalib/SplineTypes>
#include <AxCore/TimeUtils>
#include <AxMath/MathBase>
#include <AxMVS/Landmark>
#include <AxMVS/View>

#include "ceres_spline_residuals.h"
#include "spline_trajectory_estimator.h"

namespace thoht {

template <int Order_t>
SplineTrajectoryEstimator<Order_t>::SplineTrajectoryEstimator()
    : SplineTrajectoryEstimator<Order_t>(time::sToNs(0.1), time::sToNs(0.1), 0.)
{
}

template <int Order_t>
SplineTrajectoryEstimator<Order_t>::SplineTrajectoryEstimator(
    int64_t time_interval_so3, int64_t time_interval_r3, int64_t start_time)
    : dt_so3_ns_(time_interval_so3),
      dt_r3_ns_(time_interval_r3),
      start_t_(start_time),
      gravity_(0., 0., kGravityMagnitude),
      inv_so3_dt_(1. / time::nsToS(dt_so3_ns_)),
      inv_r3_dt_(1. / time::nsToS(dt_r3_ns_))
{
    accl_intrinsics_ << 0., 0., 0., 1., 1., 1.;
    gyro_intrinsics_ << 0., 0., 0., 0., 0., 0., 1., 1., 1.;
}

template <int Order_t>
void SplineTrajectoryEstimator<Order_t>::setScene(const Scene& scene)
{
    scene_ = scene;

    // Calculate all reference bearings
    //    const auto trackIds = scene_.trackIds();
    //    for (const auto& trackId : trackIds) {
    //        auto* track = scene_.rTrack(trackId);
    //        const auto refViewId = track->referenceViewId();
    //        const auto* refView = scene_.view(refViewId);
    //        const Eigen::Vector2d feat =
    //        (*refView->featureOf(trackId)).point_; Eigen::Vector3d bearing =
    //            refView->camera().pixelToNormalizedCoordinates(feat);
    //        Eigen::Vector3d adjusted_point =
    //            track->position().head<3>() -
    //            track->position()[3] * refView->camera().position();
    //        Eigen::Vector3d rotated_point =
    //            refView->camera().orientationAsRotationMatrix() *
    //            adjusted_point;
    //        if (std::abs(rotated_point[2]) < 1e-10)
    //            *track->rInverseDepth() = 1 / 1e-10;
    //        else
    //            *track->rInverseDepth() = (1 / rotated_point[2]);

    //        *track->rInverseDepth() = (1 / 0.5);
    //        track->setReferenceBearing(bearing);
    //    }
}

template <int Order_t>
void SplineTrajectoryEstimator<Order_t>::setGravity(const Eigen::Vector3d& g)
{
    gravity_ = g;
}

template <int Order_t>
Eigen::Vector3d SplineTrajectoryEstimator<Order_t>::gravity() const
{
    return gravity_;
}

template <int Order_t>
void SplineTrajectoryEstimator<Order_t>::setTic(const Sophus::SE3<double>& T)
{
    T_i_c_ = T;
}

template <int Order_t>
Sophus::SE3d SplineTrajectoryEstimator<Order_t>::Tic() const
{
    return T_i_c_;
}

template <int Order_t>
void SplineTrajectoryEstimator<Order_t>::setTimes(int64_t time_interval_so3,
                                                  int64_t time_interval_r3,
                                                  int64_t start_time,
                                                  int64_t end_time)
{
    dt_so3_ns_ = time_interval_so3;
    dt_r3_ns_ = time_interval_r3;
    start_t_ = start_time;
    end_t_ = end_time;
    const int64_t duration = end_t_ - start_t_;
    num_knots_so3_ = duration / dt_so3_ns_ + N_;
    num_knots_r3_ = duration / dt_r3_ns_ + N_;
    inv_so3_dt_ = 1. / time::nsToS(dt_so3_ns_);
    inv_r3_dt_ = 1. / time::nsToS(dt_r3_ns_);
}

template <int Order_t>
void SplineTrajectoryEstimator<Order_t>::initBiasSplines(
    const Eigen::Vector3d& accl_init_bias, const Eigen::Vector3d& gyr_init_bias,
    int64_t dt_accl_bias, int64_t dt_gyro_bias, double max_accl_range,
    double max_gyro_range)
{
    max_accl_bias_range_ = max_accl_range;
    max_gyro_bias_range_ = max_gyro_range;

    dt_accl_bias_ns_ = dt_accl_bias;
    dt_gyro_bias_ns_ = dt_gyro_bias;

    inv_accl_bias_dt_ = 1. / dt_accl_bias_ns_;
    inv_gyro_bias_dt_ = 1. / dt_gyro_bias_ns_;

    const auto duration = end_t_ - start_t_;
    num_knots_accl_bias_ = duration / dt_accl_bias_ns_ + kBiasSplineOrder;
    num_knots_gyro_bias_ = duration / dt_gyro_bias_ns_ + kBiasSplineOrder;

    LOG(INFO) << "Initializing " << num_knots_accl_bias_
              << " acceleration bias knots with: " << accl_init_bias.transpose()
              << " m2/s"
              << "\n"
                 "Initializing "
              << num_knots_gyro_bias_ << " gyroscope bias knots. "
              << gyr_init_bias.transpose() << " rad/s";

    accl_bias_spline_.resize(num_knots_accl_bias_);
    for (int i = 0; i < num_knots_accl_bias_; ++i) {
        accl_bias_spline_[i] = accl_init_bias;
    }

    gyro_bias_spline_.resize(num_knots_gyro_bias_);
    for (int i = 0; i < num_knots_gyro_bias_; ++i) {
        gyro_bias_spline_[i] = gyr_init_bias;
    }
}

template <int Order_t>
void SplineTrajectoryEstimator<Order_t>::batchInitSO3R3VisPoses()
{
    so3_knots_ = SO3dList(num_knots_so3_);
    r3_knots_ = Vector3dList(num_knots_r3_);
    so3_knot_in_problem_ = std::vector(num_knots_so3_, false);
    r3_knot_in_problem_ = std::vector(num_knots_r3_, false);

    // first interpolate spline poses for imu update rate
    // create zero-based maps
    // get sorted poses
    StampedQuaterniondList quat_vis_map;
    StampedVector3dList translations_map;
    const auto viewIds = scene_.viewIds();
    for (const auto& viewId : viewIds) {
        const auto* view = scene_.view(viewId);
        const double time = view->timestamp();
        const Eigen::Quaterniond q_w_c{
            view->camera().orientationAsRotationMatrix().transpose()};
        const Sophus::SE3d T_w_c{q_w_c, view->camera().position()};
        const Sophus::SE3d T_w_i = T_w_c * T_i_c_.inverse();
        quat_vis_map[time] = T_w_i.so3().unit_quaternion();
        translations_map[time] = T_w_i.translation();
    }

    QuaterniondList quat_vis;
    std::vector<double> t_vis;
    for (const auto& q : quat_vis_map) {
        quat_vis.push_back(q.second);
        t_vis.push_back(q.first);
    }

    Vector3dList translations;
    for (const auto& t : translations_map) {
        translations.push_back(t.second);
    }

    // get time at which we want to interpolate
    std::vector<double> t_so3_spline, t_r3_spline;
    for (size_t i{0}; i < num_knots_so3_; ++i) {
        t_so3_spline.push_back(time::nsToS(i * dt_so3_ns_));
    }

    for (size_t i{0}; i < num_knots_r3_; ++i) {
        t_r3_spline.push_back(time::nsToS(i * dt_r3_ns_));
    }

    QuaterniondList interp_spline_quats;
    Vector3dList interpo_spline_trans;
    math::InterpolateQuaternions(t_vis, t_so3_spline, quat_vis,
                                 interp_spline_quats);
    math::InterpolateVector3s(t_vis, t_r3_spline, translations,
                              interpo_spline_trans);

    for (size_t i{0}; i < num_knots_so3_; ++i) {
        so3_knots_[i] = Sophus::SO3d(interp_spline_quats[i]);
    }

    for (size_t i{0}; i < num_knots_r3_; ++i) {
        r3_knots_[i] = interpo_spline_trans[i];
    }
}

template <int Order_t>
void SplineTrajectoryEstimator<Order_t>::setFixedParams(int flags)
{
    if (problem_.HasParameterBlock(T_i_c_.data())) {
        if (!(flags & SplineOptimizeType::T_I_C)) {
            problem_.SetParameterBlockConstant(T_i_c_.data());
            LOG(INFO) << "Keeping T_I_C constant.";
        }
        else {
            problem_.SetManifold(T_i_c_.data(),
                                 new Sophus::Manifold<Sophus::SE3>());
            problem_.SetParameterBlockVariable(T_i_c_.data());
            LOG(INFO) << "Optimizing T_I_C.";
        }
    }

    if (problem_.HasParameterBlock(&cam_line_delay_s_) &&
        cam_line_delay_s_ != 0.0) {
        if (!(flags & SplineOptimizeType::CAM_LINE_DELAY)) {
            problem_.SetParameterBlockConstant(&cam_line_delay_s_);
            LOG(INFO) << "Keeping camera line delay constant at: "
                      << cam_line_delay_s_;
        }
        else {
            problem_.SetParameterBlockVariable(&cam_line_delay_s_);
            LOG(INFO) << "Optimizing camera line delay.";
        }
    }

    // if IMU to Cam trafo should be optimized
    if (problem_.HasParameterBlock(gravity_.data())) {
        if (!(flags & SplineOptimizeType::GRAVITY_DIR)) {
            LOG(INFO) << "Keeping gravity direction constant at: "
                      << gravity_.transpose();

            problem_.SetParameterBlockConstant(gravity_.data());
        }
        else {
            if (problem_.HasParameterBlock(gravity_.data()))
                problem_.SetParameterBlockVariable(gravity_.data());
            LOG(INFO) << "Optimizing gravity direction.";
        }
    }

    // if world points should be optimized
    if (!(flags & SplineOptimizeType::POINTS)) {
        LOG(INFO) << "Keeping object points constant.";
        for (const auto& tid : tracks_in_problem_) {
            const auto track = scene_.rTrack(tid)->rPosition().data();
            if (problem_.HasParameterBlock(track))
                problem_.SetParameterBlockConstant(track);
        }
    }
    else {
        for (const auto& tid : tracks_in_problem_) {
            const auto track = scene_.rTrack(tid)->rPosition().data();
            if (problem_.HasParameterBlock(track)) {
                problem_.SetParameterBlockVariable(track);
                problem_.SetManifold(track, new ceres::SphereManifold<4>());
            }
        }
        LOG(INFO) << "Optimizing object points.";
    }

    // if imu intrinics should be optimized
    if (problem_.HasParameterBlock(accl_intrinsics_.data()) &&
        problem_.HasParameterBlock(gyro_intrinsics_.data())) {
        if (!(flags & SplineOptimizeType::IMU_INTRINSICS)) {
            LOG(INFO) << "Keeping IMU intrinsics constant.";
            problem_.SetParameterBlockConstant(accl_intrinsics_.data());
            problem_.SetParameterBlockConstant(gyro_intrinsics_.data());
        }
        else {
            problem_.SetParameterBlockVariable(accl_intrinsics_.data());
            problem_.SetParameterBlockVariable(gyro_intrinsics_.data());
            LOG(INFO) << "Optimizing IMU intrinsics.";
        }
    }

    // add local parametrization for SO(3)
    for (auto& knot : so3_knots_) {
        if (problem_.HasParameterBlock(knot.data())) {
            problem_.SetManifold(knot.data(),
                                 new Sophus::Manifold<Sophus::SO3>());
        }
    }

    if (!(flags & SplineOptimizeType::SPLINE)) {
        // set knots constant if asked
        for (const auto& knot : r3_knots_) {
            if (problem_.HasParameterBlock(knot.data())) {
                problem_.SetParameterBlockConstant(knot.data());
            }
        }
        for (const auto& knot : so3_knots_) {
            if (problem_.HasParameterBlock(knot.data())) {
                problem_.SetParameterBlockConstant(knot.data());
            }
        }
    }
    else {
        // Set knots constant if asked
        for (auto& knot : r3_knots_) {
            if (problem_.HasParameterBlock(knot.data())) {
                problem_.SetParameterBlockVariable(knot.data());
            }
        }
        for (auto& knot : so3_knots_) {
            if (problem_.HasParameterBlock(knot.data())) {
                problem_.SetParameterBlockVariable(knot.data());
            }
        }
    }

    // Set bias limits
    // If IMU intrinics should be optimized
    if ((flags & SplineOptimizeType::ACC_BIAS ||
         flags & SplineOptimizeType::IMU_BIASES)) {
        LOG(INFO) << "Optimizing accelerometer bias spline.";
        for (auto& acc : accl_bias_spline_) {
            if (problem_.HasParameterBlock(acc.data())) {
                for (int dim{0}; dim < 3; ++dim) {
                    problem_.SetParameterLowerBound(acc.data(), dim,
                                                    -max_accl_bias_range_);
                    problem_.SetParameterUpperBound(acc.data(), dim,
                                                    max_accl_bias_range_);
                }
                problem_.SetParameterBlockVariable(acc.data());
            }
        }
    }
    else {
        LOG(INFO) << "Fixing accelerometer bias spline.";
        for (const auto& bias : accl_bias_spline_) {
            if (problem_.HasParameterBlock(bias.data())) {
                problem_.SetParameterBlockConstant(bias.data());
            }
        }
    }

    if ((flags & SplineOptimizeType::GYR_BIAS ||
         flags & SplineOptimizeType::IMU_BIASES)) {
        LOG(INFO) << "Optimizing gyroscope bias spline.";
        for (auto& bias : gyro_bias_spline_) {
            if (problem_.HasParameterBlock(bias.data())) {
                for (int dim{0}; dim < 3; ++dim) {
                    problem_.SetParameterLowerBound(bias.data(), dim,
                                                    -max_gyro_bias_range_);
                    problem_.SetParameterUpperBound(bias.data(), dim,
                                                    max_gyro_bias_range_);
                }
                problem_.SetParameterBlockVariable(bias.data());
            }
        }
    }
    else {
        LOG(INFO) << "Fixing gyroscope bias spline.";
        for (const auto& bias : gyro_bias_spline_) {
            if (problem_.HasParameterBlock(bias.data())) {
                problem_.SetParameterBlockConstant(bias.data());
            }
        }
    }
}

template <int Order_t>
ceres::Solver::Summary SplineTrajectoryEstimator<Order_t>::optimize(
    int max_iters, int flags)
{
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = max_iters;
    options.num_threads = std::thread::hardware_concurrency();
    options.minimizer_progress_to_stdout = true;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.function_tolerance = 1e-4;
    options.parameter_tolerance = 1e-7;
    options.preconditioner_type = ceres::CLUSTER_TRIDIAGONAL;
    options.use_inner_iterations = true;

    setFixedParams(flags);

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem_, &summary);
    LOG(INFO) << summary.FullReport();

    return summary;
}

template <int Order_t>
bool SplineTrajectoryEstimator<Order_t>::addAccelerometerMeasurement(
    const Eigen::Vector3d& meas, int64_t time, double weight_se3)
{
    double u_r3, u_so3, u_bias;
    int64_t s_r3, s_so3, s_bias;
    if (!calcR3Times(time, u_r3, s_r3)) {
        LOG(INFO)
            << "Wrong time adding r3 accelerometer measurements. time_ns: "
            << time << " u_r3: " << u_r3 << " s_r3:" << s_r3;
        return false;
    }
    if (!calcSO3Times(time, u_so3, s_so3)) {
        LOG(INFO)
            << "Wrong time adding so3 accelerometer measurements. time_ns: "
            << time << " u_r3: " << u_r3 << " s_r3:" << s_r3;
        return false;
    }
    if (!calcTimes(time, u_bias, s_bias, dt_accl_bias_ns_, num_knots_accl_bias_,
                   kBiasSplineOrder)) {
        LOG(INFO)
            << "Wrong time adding accelerometer bias measurements. time_ns: "
            << time << " u_r3: " << u_bias << " s_r3:" << s_bias;
        return false;
    }

    using FunctorT = AccelerationCostFunctorSplit<N_>;
    auto* functor = new FunctorT(meas, u_r3, inv_r3_dt_, u_so3, inv_so3_dt_,
                                 weight_se3, u_bias, inv_accl_bias_dt_);

    auto* cost_function =
        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

    std::vector<double*> vec;
    // so3 spline
    for (int i = 0; i < N_; i++) {
        cost_function->AddParameterBlock(4);
        const int t = s_so3 + i;
        vec.emplace_back(so3_knots_[t].data());
        so3_knot_in_problem_[t] = true;
    }

    // R3 spline
    for (int i = 0; i < N_; i++) {
        cost_function->AddParameterBlock(3);
        const int t = s_r3 + i;
        vec.emplace_back(r3_knots_[t].data());
        r3_knot_in_problem_[t] = true;
    }

    // bias spline
    for (int i = 0; i < kBiasSplineOrder; i++) {
        cost_function->AddParameterBlock(3);
        const int t = s_bias + i;
        vec.emplace_back(accl_bias_spline_[t].data());
    }

    // Gravity
    cost_function->AddParameterBlock(3);
    vec.emplace_back(gravity_.data());

    // IMU intrinsics and bias
    cost_function->AddParameterBlock(6);
    vec.emplace_back(accl_intrinsics_.data());

    // number of residuals
    cost_function->SetNumResiduals(3);

    problem_.AddResidualBlock(cost_function, nullptr, vec);

    return true;
}

template <int Order_t>
bool SplineTrajectoryEstimator<Order_t>::addGyroscopeMeasurement(
    const Eigen::Vector3d& meas, int64_t time, double weight_so3)
{
    double u_so3, u_bias;
    int64_t s_so3, s_bias;
    if (!calcSO3Times(time, u_so3, s_so3)) {
        LOG(INFO) << "Wrong time adding so3 gyroscope measurements. time_ns: "
                  << time << " u_r3: " << u_so3 << " s_r3:" << s_so3;
        return false;
    }

    if (!calcTimes(time, u_bias, s_bias, dt_gyro_bias_ns_, num_knots_gyro_bias_,
                   kBiasSplineOrder)) {
        LOG(INFO)
            << "Wrong time adding so3 gyroscope bias measurements. time_ns: "
            << time << " u_r3: " << u_bias << " s_r3:" << s_bias;
        return false;
    }

    using FunctorT = GyroCostFunctorSplit<N_, Sophus::SO3, false>;
    auto* functor = new FunctorT(meas, u_so3, inv_so3_dt_, weight_so3, u_bias,
                                 inv_gyro_bias_dt_);

    auto* cost_function =
        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

    // SO3 spline
    std::vector<double*> vec;
    for (int i = 0; i < N_; i++) {
        cost_function->AddParameterBlock(4);
        const int t = s_so3 + i;
        vec.emplace_back(so3_knots_[t].data());
        so3_knot_in_problem_[t] = true;
    }
    // bias spline
    for (int i = 0; i < kBiasSplineOrder; ++i) {
        cost_function->AddParameterBlock(3);
        const int t = s_bias + i;
        vec.emplace_back(gyro_bias_spline_[t].data());
    }
    // intrinsics
    cost_function->AddParameterBlock(9);
    vec.emplace_back(gyro_intrinsics_.data());

    cost_function->SetNumResiduals(3);

    problem_.AddResidualBlock(cost_function, nullptr, vec);

    return true;
}

template <int _T>
bool SplineTrajectoryEstimator<_T>::addGSCameraMeasurement(
    const View* view, double robust_loss_width)
{
    const int64_t image_obs_time = time::sToNs(view->timestamp());
    const auto track_ids = view->trackIds();

    double u_r3 = 0.0, u_so3 = 0.0;
    int64_t s_r3 = 0, s_so3 = 0;
    if (!calcR3Times(image_obs_time, u_r3, s_r3)) {
        LOG(INFO) << "Wrong time observation r3 vision measurements. time_ns: "
                  << image_obs_time << " u_r3: " << u_r3 << " s_r3:" << s_r3;
        return false;
    }
    if (!calcSO3Times(image_obs_time, u_so3, s_so3)) {
        LOG(INFO) << "Wrong time reference so3 vision measurements. time_ns: "
                  << image_obs_time << " u_r3: " << u_so3 << " s_r3:" << s_so3;
        return false;
    }

    using FunctorT = GSReprojectionCostFunctorSplit<N_>;
    auto* functor = new FunctorT(view, &scene_, u_so3, u_r3, inv_so3_dt_,
                                 inv_r3_dt_, track_ids);

    auto* cost_function =
        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

    std::vector<double*> vec;
    for (int i = 0; i < N_; i++) {
        cost_function->AddParameterBlock(4);
        const int t = s_so3 + i;
        vec.emplace_back(so3_knots_[t].data());
        so3_knot_in_problem_[t] = true;
    }
    for (int i = 0; i < N_; i++) {
        cost_function->AddParameterBlock(3);
        const int t = s_r3 + i;
        vec.emplace_back(r3_knots_[t].data());
        r3_knot_in_problem_[t] = true;
    }

    // camera to imu transformation
    cost_function->AddParameterBlock(7);
    vec.emplace_back(T_i_c_.data());

    // object point
    for (const auto& id : track_ids) {
        cost_function->AddParameterBlock(4);
        vec.emplace_back(scene_.rTrack(id)->rPosition().data());
        tracks_in_problem_.insert(id);
    }

    cost_function->SetNumResiduals(track_ids.size() * 2);

    auto* loss_function = new ceres::HuberLoss(robust_loss_width);
    problem_.AddResidualBlock(cost_function, loss_function, vec);

    return true;
}

template <int _T>
bool SplineTrajectoryEstimator<_T>::addRSCameraMeasurement(
    const View* view, double robust_loss_width)
{
    const auto image_obs_time = time::sToNs(view->timestamp());
    const auto trackIds = view->trackIds();

    double u_r3 = 0.0, u_so3 = 0.0;
    int64_t s_r3 = 0, s_so3 = 0;
    if (!calcR3Times(image_obs_time, u_r3, s_r3)) {
        LOG(INFO) << "Wrong time observation r3 vision measurements. time_ns: "
                  << image_obs_time << " u_r3: " << u_r3 << " s_r3:" << s_r3;
        return false;
    }
    if (!calcSO3Times(image_obs_time, u_so3, s_so3)) {
        LOG(INFO) << "Wrong time reference so3 vision measurements. time_ns: "
                  << image_obs_time << " u_r3: " << u_so3 << " s_r3:" << s_so3;
        return false;
    }

    using FunctorT = RSReprojectionCostFunctorSplit<N_>;
    auto* functor = new FunctorT(view, &scene_, u_so3, u_r3, inv_so3_dt_,
                                 inv_r3_dt_, trackIds);

    auto* cost_function =
        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

    std::vector<double*> vec;
    for (int i = 0; i < N_; i++) {
        cost_function->AddParameterBlock(4);
        const int t = s_so3 + i;
        vec.emplace_back(so3_knots_[t].data());
        so3_knot_in_problem_[t] = true;
    }
    for (int i = 0; i < N_; i++) {
        cost_function->AddParameterBlock(3);
        const int t = s_r3 + i;
        vec.emplace_back(r3_knots_[t].data());
        r3_knot_in_problem_[t] = true;
    }

    // camera to imu transformation
    cost_function->AddParameterBlock(7);
    vec.emplace_back(T_i_c_.data());

    // line delay for rolling shutter cameras
    cost_function->AddParameterBlock(1);
    vec.emplace_back(&cam_line_delay_s_);

    // object point
    for (const auto& id : trackIds) {
        cost_function->AddParameterBlock(4);
        vec.emplace_back(scene_.rTrack(id)->rPosition().data());
        tracks_in_problem_.insert(id);
    }

    cost_function->SetNumResiduals(trackIds.size() * 2);

    if (robust_loss_width == 0.0) {
        problem_.AddResidualBlock(cost_function, nullptr, vec);
    }
    else {
        auto* loss_function = new ceres::HuberLoss(robust_loss_width);
        problem_.AddResidualBlock(cost_function, loss_function, vec);
    }

    // bound translation
    //  problem_.SetParameterLowerBound(T_i_c_.data(), 4, -1e-2);
    //  problem_.SetParameterUpperBound(T_i_c_.data(), 4, 1e-2);
    //  problem_.SetParameterLowerBound(T_i_c_.data(), 5, -10e-2);
    //  problem_.SetParameterUpperBound(T_i_c_.data(), 5, 10e-2);
    //  problem_.SetParameterLowerBound(T_i_c_.data(), 6, -1e-2);
    //  problem_.SetParameterUpperBound(T_i_c_.data(), 6, 1e-2);

    return true;
}

template <int _T>
bool SplineTrajectoryEstimator<_T>::calcTimes(int64_t sensor_time, double& u,
                                              int64_t& s, int64_t dt,
                                              size_t nr_knots, int N) const
{
    const int64_t st_ns = sensor_time - start_t_;
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

template <int _T>
bool SplineTrajectoryEstimator<_T>::calcSO3Times(int64_t sensor_time,
                                                 double& u_so3,
                                                 int64_t& s_so3) const
{
    return calcTimes(sensor_time, u_so3, s_so3, dt_so3_ns_, so3_knots_.size());
}

template <int _T>
bool SplineTrajectoryEstimator<_T>::calcR3Times(int64_t sensor_time,
                                                double& u_r3,
                                                int64_t& s_r3) const
{
    return calcTimes(sensor_time, u_r3, s_r3, dt_r3_ns_, r3_knots_.size());
}

template <int _T>
Sophus::SE3d SplineTrajectoryEstimator<_T>::knotAt(int i) const
{
    return {so3_knots_[i], r3_knots_[i]};
}

template <int _T>
size_t SplineTrajectoryEstimator<_T>::GetNumSO3Knots() const
{
    return so3_knots_.size();
}

template <int _T>
size_t SplineTrajectoryEstimator<_T>::GetNumR3Knots() const
{
    return r3_knots_.size();
}

template <int _T>
int64_t SplineTrajectoryEstimator<_T>::GetMaxTimeNs() const
{
    return start_t_ + (so3_knots_.size() - N_ + 1) * dt_so3_ns_ - 1;
}

template <int _T>
int64_t SplineTrajectoryEstimator<_T>::GetMinTimeNs() const
{
    return start_t_;
}

template <int _T>
void SplineTrajectoryEstimator<_T>::setImuToCameraTimeOffset(double offset)
{
    imu_to_camera_time_offset_s_ = offset;
}

template <int _T>
void SplineTrajectoryEstimator<_T>::setCameraLineDelay(
    const double cam_line_delay_s)
{
    cam_line_delay_s_ = cam_line_delay_s;
}

template <int _T>
bool SplineTrajectoryEstimator<_T>::poseAt(int64_t time,
                                           Sophus::SE3d& pose) const
{
    double u_r3;
    int64_t s_r3;
    if (!calcR3Times(time, u_r3, s_r3)) {
        return false;
    }

    double u_so3;
    int64_t s_so3;
    if (!calcSO3Times(time, u_so3, s_so3)) {
        return false;
    }

    Sophus::SO3d rot;
    {
        std::vector<const double*> vec;
        for (int i = 0; i < N_; ++i) {
            vec.emplace_back(so3_knots_[s_so3 + i].data());
        }

        CeresSplineHelper<double, N_>::template evaluate_lie<Sophus::SO3>(
            &vec[0], u_so3, inv_so3_dt_, &rot);
    }

    Eigen::Vector3d trans;
    {
        std::vector<const double*> vec;
        for (int i = 0; i < N_; ++i) {
            vec.emplace_back(r3_knots_[s_r3 + i].data());
        }

        CeresSplineHelper<double, N_>::template evaluate<3, 0>(
            &vec[0], u_r3, inv_r3_dt_, &trans);
    }

    pose = {rot, trans};
    return true;
}

template <int _T>
bool SplineTrajectoryEstimator<_T>::GetPosition(int64_t time_ns,
                                                Eigen::Vector3d& position) const
{
    double u_r3;
    int64_t s_r3;
    if (!calcR3Times(time_ns, u_r3, s_r3)) {
        return false;
    }

    std::vector<const double*> vec;
    for (int i{0}; i < N_; ++i) {
        vec.emplace_back(r3_knots_[s_r3 + i].data());
    }

    CeresSplineHelper<double, N_>::template evaluate<3, 0>(
        &vec[0], u_r3, inv_r3_dt_, &position);

    return true;
}

template <int _T>
bool SplineTrajectoryEstimator<_T>::GetAngularVelocity(
    int64_t time_ns, Eigen::Vector3d& velocity) const
{
    double u_so3;
    int64_t s_so3;

    if (!calcSO3Times(time_ns, u_so3, s_so3)) {
        return false;
    }

    std::vector<const double*> vec;
    for (int i = 0; i < N_; ++i) {
        vec.emplace_back(so3_knots_[s_so3 + i].data());
    }

    CeresSplineHelper<double, N_>::template evaluate_lie<Sophus::SO3>(
        &vec[0], u_so3, inv_so3_dt_, nullptr, &velocity);

    return true;
}

template <int _T>
bool SplineTrajectoryEstimator<_T>::GetAcceleration(
    int64_t time_ns, Eigen::Vector3d& acceleration) const
{
    double u_r3;
    int64_t s_r3;
    if (!calcR3Times(time_ns, u_r3, s_r3)) {
        return false;
    }

    double u_so3;
    int64_t s_so3;
    if (!calcSO3Times(time_ns, u_so3, s_so3)) {
        return false;
    }

    Sophus::SO3d rot;
    {
        std::vector<const double*> vec;
        for (int i = 0; i < N_; ++i) {
            vec.emplace_back(so3_knots_[s_so3 + i].data());
        }

        CeresSplineHelper<double, N_>::template evaluate_lie<Sophus::SO3>(
            &vec[0], u_so3, inv_so3_dt_, &rot);
    }

    Eigen::Vector3d trans_accel_world;
    {
        std::vector<const double*> vec;
        for (int i = 0; i < N_; ++i) {
            vec.emplace_back(r3_knots_[s_r3 + i].data());
        }

        CeresSplineHelper<double, N_>::template evaluate<3, 2>(
            &vec[0], u_r3, inv_r3_dt_, &trans_accel_world);
    }

    acceleration = rot.inverse() * (trans_accel_world + gravity_);
    return true;
}

template <int _T>
double SplineTrajectoryEstimator<_T>::calcMeanReprojectionError()
{
    // ConvertInvDepthPointsToHom();
    double avgRPE{0.};
    int num_points{0};
    for (const auto& viewId : scene_.viewIds()) {
        const auto* view = scene_.view(viewId);
        const auto trackIds = view->trackIds();
        const size_t n_obs = trackIds.size();
        if (n_obs == 0) {
            return false;
        }

        const int64_t image_time = time::sToNs(view->timestamp());

        double u_r3, u_so3;
        int64_t s_r3, s_so3;
        if (!calcR3Times(image_time, u_r3, s_r3)) {
            return 0.;
        }
        if (!calcSO3Times(image_time, u_so3, s_so3)) {
            return 0.;
        }

        using CostFunctor = RSReprojectionCostFunctorSplit<N_>;
        auto* functor = new CostFunctor(view, &scene_, u_so3, u_r3, inv_so3_dt_,
                                        inv_r3_dt_, trackIds);

        auto* costFunc =
            new ceres::DynamicAutoDiffCostFunction<CostFunctor>(functor);

        std::vector<double*> vec;
        for (int i = 0; i < N_; i++) {
            costFunc->AddParameterBlock(4);
            const int t = s_so3 + i;
            vec.emplace_back(so3_knots_[t].data());
        }
        for (int i = 0; i < N_; i++) {
            costFunc->AddParameterBlock(3);
            const int t = s_r3 + i;
            vec.emplace_back(r3_knots_[t].data());
        }

        // camera to imu transformation
        costFunc->AddParameterBlock(7);
        vec.emplace_back(T_i_c_.data());

        // line delay for rolling shutter cameras
        costFunc->AddParameterBlock(1);
        vec.emplace_back(&cam_line_delay_s_);

        // all object points
        for (size_t i = 0; i < n_obs; ++i) {
            costFunc->AddParameterBlock(4);
            vec.emplace_back(scene_.rTrack(trackIds[i])->rPosition().data());
        }

        costFunc->SetNumResiduals(2 * n_obs);
        {
            Eigen::VectorXd residual;
            residual.setZero(n_obs * 2);

            costFunc->Evaluate(&vec[0], residual.data(), nullptr);

            for (size_t i = 0; i < n_obs; i++) {
                Eigen::Vector2d res_point = residual.segment<2>(2 * i);
                if (res_point[0] != 0.0 && res_point[1] != 0.0) {
                    avgRPE += res_point.norm();
                    num_points += 1;
                }
            }
        }
    }

    double mean_rpe = avgRPE / num_points;

    LOG(INFO) << "Mean reprojection error " << mean_rpe
              << " number residuals: " << num_points;

    return mean_rpe;
}

template <int _T>
void SplineTrajectoryEstimator<_T>::ConvertInvDepthPointsToHom()
{
    for (const auto& id : scene_.trackIds()) {
        auto* track = scene_.rTrack(id);
        const auto* ref_view = scene_.view(track->referenceViewId());
        Eigen::Vector3d bearing = ref_view->camera().pixelToUnitDepthRay(
            ref_view->featureOf(id)->point_);

        const auto inverseDepth = *track->inverseDepth();

        // 1.
        const auto time = time::sToNs(ref_view->timestamp());
        Sophus::SE3d T_w_i;
        poseAt(time, T_w_i);
        Eigen::Vector3d X_ref =
            T_i_c_.so3() * (bearing - inverseDepth * T_i_c_.translation());

        // 2. Transform point from IMU to world frame
        Eigen::Vector3d X =
            T_w_i.so3() * X_ref + T_w_i.translation() * inverseDepth;
        track->rPosition() = X.homogeneous();
        track->setEstimated(true);
    }
}

template <int _T>
void SplineTrajectoryEstimator<_T>::ConvertToTheiaRecon(Scene* recon_out)
{
    // read camera calibration
    for (const auto& viewId : scene_.viewIds()) {
        const auto time = time::sToNs(scene_.view(viewId)->timestamp());
        Sophus::SE3d T_w_i;
        poseAt(time, T_w_i);
        Sophus::SE3d T_w_c = T_w_i * T_i_c_;

        const auto newViewId =
            recon_out->addView(std::to_string(time), CameraId{0}, time);
        auto* view = recon_out->rView(newViewId);
        view->setEstimated(true);
        auto& camera = view->rCamera();
        camera.setOrientationFromRotationMatrix(
            T_w_c.rotationMatrix().transpose());
        camera.setPosition(T_w_c.translation());
    }

    // ConvertInvDepthPointsToHom();
    for (const auto& id : scene_.trackIds()) {
        auto tid = recon_out->addTrack();
        recon_out->rTrack(tid)->rPosition() = scene_.track(id)->position();
        recon_out->rTrack(tid)->setEstimated(true);
    }
}

template <int _T>
double SplineTrajectoryEstimator<_T>::cameraLineDelay() const
{
    return cam_line_delay_s_;
}

template <int _T>
ImuIntrinsics SplineTrajectoryEstimator<_T>::accelerometerIntrinsicsAt(
    int64_t time) const
{
    const auto bias = GetAcclBias(time);
    ImuIntrinsics intrinsics{accl_intrinsics_[0],
                             accl_intrinsics_[1],
                             accl_intrinsics_[2],
                             0.,
                             0.,
                             0.,
                             accl_intrinsics_[3],
                             accl_intrinsics_[4],
                             accl_intrinsics_[5],
                             bias[0],
                             bias[1],
                             bias[2]};
    return intrinsics;
}

template <int _T>
ImuIntrinsics SplineTrajectoryEstimator<_T>::gyroscopeIntrinsicsAt(
    int64_t time) const
{
    const auto bias = GetAcclBias(time);
    ImuIntrinsics intrinsics{gyro_intrinsics_[0],
                             gyro_intrinsics_[1],
                             gyro_intrinsics_[2],
                             gyro_intrinsics_[3],
                             gyro_intrinsics_[4],
                             gyro_intrinsics_[5],
                             gyro_intrinsics_[6],
                             gyro_intrinsics_[7],
                             gyro_intrinsics_[8],
                             bias[0],
                             bias[1],
                             bias[2]};
    return intrinsics;
}

template <int _T>
Eigen::Vector3d SplineTrajectoryEstimator<_T>::GetGyroBias(int64_t time) const
{
    double u;
    int64_t s;
    Eigen::Vector3d bias;
    bias.setZero();
    if (!calcTimes(time, u, s, dt_gyro_bias_ns_, num_knots_gyro_bias_,
                   kBiasSplineOrder)) {
        return bias;
    }

    std::vector<const double*> vec;
    for (int i = 0; i < kBiasSplineOrder; ++i) {
        vec.emplace_back(gyro_bias_spline_[s + i].data());
    }

    CeresSplineHelper<double, kBiasSplineOrder>::template evaluate<3, 0>(
        &vec[0], u, inv_gyro_bias_dt_, &bias);

    return bias;
}

template <int _T>
Eigen::Vector3d SplineTrajectoryEstimator<_T>::GetAcclBias(int64_t time) const
{
    double u;
    int64_t s;
    Eigen::Vector3d bias;
    bias.setZero();
    if (!calcTimes(time, u, s, dt_accl_bias_ns_, num_knots_accl_bias_,
                   kBiasSplineOrder)) {
        return bias;
    }

    std::vector<const double*> vec;
    for (int i = 0; i < kBiasSplineOrder; ++i) {
        vec.emplace_back(accl_bias_spline_[s + i].data());
    }

    CeresSplineHelper<double, kBiasSplineOrder>::template evaluate<3, 0>(
        &vec[0], u, inv_accl_bias_dt_, &bias);

    return bias;
}

template <int _T>
void SplineTrajectoryEstimator<_T>::setIMUIntrinsics(
    const ImuIntrinsics& accl_intrinsics, const ImuIntrinsics& gyro_intrinsics)
{
    accl_intrinsics_ << accl_intrinsics.misYZ(), accl_intrinsics.misZY(),
        accl_intrinsics.misZX(), accl_intrinsics.scaleX(),
        accl_intrinsics.scaleY(), accl_intrinsics.scaleZ();

    gyro_intrinsics_ << gyro_intrinsics.misYZ(), gyro_intrinsics.misZY(),
        gyro_intrinsics.misZX(), gyro_intrinsics.misXZ(),
        gyro_intrinsics.misXY(), gyro_intrinsics.misYX(),
        gyro_intrinsics.scaleX(), gyro_intrinsics.scaleY(),
        gyro_intrinsics.scaleZ();
}

} // namespace thoht
