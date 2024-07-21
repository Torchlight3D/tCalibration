#pragma once

#include <cstdint>

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <tMath/Eigen/Utils>
#include <tMotion/ImuData>
#include <tMotion/ImuIntrinsics>
#include <tMvs/View>
#include <tMvs/Scene>

namespace tl {

using SO3dList = eigen_vector<Sophus::SO3d>;
using SE3dList = eigen_vector<Sophus::SE3d>;

inline constexpr double kGravityMagnitude{9.81};

template <int _N>
class SplineTrajectoryEstimator
{
public:
    static constexpr int N_ = _N;       // Order of the spline.
    static constexpr int DEG_ = _N - 1; // Degree of the spline.

    SplineTrajectoryEstimator();
    SplineTrajectoryEstimator(int64_t time_interval_so3,
                              int64_t time_interval_r3, int64_t start_time);

    /// Data
    void setGravity(const Eigen::Vector3d& gravity);
    Eigen::Vector3d gravity() const;

    void setTic(const Sophus::SE3d& imuToCameraTransform);
    Sophus::SE3d Tic() const;

    void setCameraLineDelay(double delay);
    double cameraLineDelay() const;

    void setScene(const Scene& scene);

    bool addAccelerometerMeasurement(const Eigen::Vector3d& measurements,
                                     int64_t timestamp, double weight);
    bool addGyroscopeMeasurement(const Eigen::Vector3d& measurements,
                                 int64_t timestamp, double weight);

    bool addGSCameraMeasurement(const View* view, double robust_loss_width);
    bool addRSCameraMeasurement(const View* view,
                                double robust_loss_width = 0.);

    void setIMUIntrinsics(const ImuIntrinsics& acclIntrinsics,
                          const ImuIntrinsics& gyroIntrinsics);

    /// Configs
    void setTimes(int64_t time_interval_so3, int64_t time_interval_r3,
                  int64_t start_time, int64_t end_time);

    void initBiasSplines(const Eigen::Vector3d& accl_init_bias,
                         const Eigen::Vector3d& gyr_init_bias,
                         int64_t dt_accl_bias = 500000000,
                         int64_t dt_gyro_bias = 500000000,
                         double max_accl_range = 1.0,
                         double max_gyro_range = 1e-2);

    void batchInitSO3R3VisPoses();

    /// Actions
    ceres::Solver::Summary optimize(int max_iters, int flags);

    /// Results
    bool poseAt(int64_t timestamp, Sophus::SE3d& pose) const;
    Eigen::Vector3d GetAcclBias(int64_t time) const;
    Eigen::Vector3d GetGyroBias(int64_t time) const;

    double calcMeanReprojectionError();

    ImuIntrinsics accelerometerIntrinsicsAt(int64_t timestamp) const;
    ImuIntrinsics gyroscopeIntrinsicsAt(int64_t timestamp) const;

private:
    void setFixedParams(int flags);

    void setImuToCameraTimeOffset(double imu_to_camera_time_offset);

    Sophus::SE3d knotAt(int index) const;

    size_t GetNumSO3Knots() const;
    size_t GetNumR3Knots() const;

    int64_t GetMaxTimeNs() const;
    int64_t GetMinTimeNs() const;

    bool GetPosition(int64_t time_ns, Eigen::Vector3d& position) const;

    bool GetAngularVelocity(int64_t time, Eigen::Vector3d& velocity) const;
    bool GetAcceleration(int64_t time, Eigen::Vector3d& acceleration) const;

    void toScene(Scene* recon_out) const;
    void ConvertInvDepthPointsToHom();

    bool calcSO3Times(int64_t sensor_time, double& u_so3, int64_t& s_so3) const;
    bool calcR3Times(int64_t sensor_time, double& u_r3, int64_t& s_r3) const;
    bool calcTimes(int64_t sensor_time, double& u, int64_t& s, int64_t dt,
                   size_t nr_knots, int N = N_) const;

private:
    int64_t start_t_;
    int64_t end_t_;

    int64_t dt_so3_ns_;
    int64_t dt_r3_ns_;

    double inv_r3_dt_;
    double inv_so3_dt_;

    size_t num_knots_so3_;
    size_t num_knots_r3_;

    SO3dList so3_knots_;
    Vector3dList r3_knots_;

    std::vector<bool> so3_knot_in_problem_;
    std::vector<bool> r3_knot_in_problem_;

    //! bias spline meta data
    size_t num_knots_accl_bias_;
    size_t num_knots_gyro_bias_;

    int64_t dt_accl_bias_ns_;
    int64_t dt_gyro_bias_ns_;

    double inv_accl_bias_dt_;
    double inv_gyro_bias_dt_;

    Vector3dList gyro_bias_spline_;
    Vector3dList accl_bias_spline_;

    double max_accl_bias_range_ = 1.0;
    double max_gyro_bias_range_ = 1e-2;

    //! parameters
    int optim_flags_;

    bool fix_imu_intrinsics_ = false;

    double cam_line_delay_s_ = 0.0;

    double imu_to_camera_time_offset_s_ = 0.0;

    std::set<int> tracks_in_problem_;

    Eigen::Vector3d gravity_;

    // half of misalignment and scale
    Eigen::Matrix<double, 6, 1> accl_intrinsics_;
    // all misalignment and scale
    Eigen::Matrix<double, 9, 1> gyro_intrinsics_;

    Scene scene_;

    Sophus::SE3<double> T_i_c_;

    ceres::Problem problem_;

    bool spline_initialized_with_gps_ = false;
};

inline bool KnotInBlock(const std::vector<double*> vec, double* knot_ptr)
{
    return std::any_of(vec.begin(), vec.end(), [&knot_ptr](const auto& val) {
        return val == knot_ptr;
    });
}

inline int GetPtrOffset(const double* knot_ptr,
                        const std::vector<double*>& container)
{
    for (size_t i = 0; i < container.size(); ++i) {
        if (knot_ptr == container[i]) {
            return i;
        }
    }
    // this should not happen!
    return 0;
}

} // namespace tl

#include "splinetrajectoryestimator.hpp"
