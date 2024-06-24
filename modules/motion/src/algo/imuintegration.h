#pragma once

#include <ceres/rotation.h>

#include <Eigen/Core>

#include <tMath/Eigen/Utils>
#include <tMotion/ImuData>

namespace tl {

template <typename T>
void IntegrateGyroIntervalbyMidPoint(const ImuReadings_<T>& samples,
                                     Eigen::Quaternion<T>& quat,
                                     const DataInterval& interval = {},
                                     double fixed_dt = -1.)
{
    const auto valid_interval = samples.validInterval(interval);
    quat = Eigen::Quaternion<T>::Identity();
    for (int i{valid_interval.start}; i < valid_interval.end; i++) {
        double dt = fixed_dt > 0.
                        ? fixed_dt
                        : samples[i + 1].timestamp() - samples[i].timestamp();

        math::QuaternionIntegrationByMidPoint(quat, samples[i].data(),
                                              samples[i + 1].data(), dt, quat);
    }
}

// Brief:
// Integrate a sequence of rotational velocities using the Runge-Kutta (4 order)
// discrete integration method. The initial rotation is assumed to be the
// identity quaternion.
//
// Input:
//     samples: Input gyroscope signal (rotational velocity samples vector)
//     fixed_dt: Fixed time step (t1 - t0) between samples. If is -1, the sample
//     timestamps are used instead.
//     interval: Data interval where to compute the integration. If this
//     interval is not valid, i.e., one of the two indices is -1, the
//     integration is computed for the whole data sequence.
// Output:
//     quat_res: Resulting final rotation quaternion
template <typename T>
void IntegrateGyroInterval(const ImuReadings_<T>& samples,
                           Eigen::Vector4<T>& quat, double fixed_dt = -1.,
                           const DataInterval& interval = {})
{
    const auto valid_interval = samples.validInterval(interval);
    quat = {T(1), T(0), T(0), T(0)};
    for (int i{valid_interval.start}; i < valid_interval.end; i++) {
        double dt = fixed_dt > 0.
                        ? fixed_dt
                        : samples[i + 1].timestamp() - samples[i].timestamp();

        math::QuaternionIntegrationByRK4(quat, samples[i].asVector(),
                                         samples[i + 1].asVector(), dt, quat);
    }
}

template <typename T>
inline void IntegrateGyroInterval(const ImuReadings_<T>& samples,
                                  Eigen::Matrix3<T>& rotation,
                                  double fixed_dt = -1.,
                                  const DataInterval& interval = {})
{
    Eigen::Vector4<T> quat;
    IntegrateGyroInterval(samples, quat, fixed_dt, interval);
    auto rmat = ceres::ColumnMajorAdapter3x3(rotation.data());
    ceres::QuaternionToRotation(quat.data(), rmat);
}

} // namespace tl
