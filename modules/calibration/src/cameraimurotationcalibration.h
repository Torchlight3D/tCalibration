#pragma once

#include <memory>

#include <tMath/Eigen/Utils>

namespace tl {

class [[deprecated("This class is deprecated, use InertialBasedScaleEstimation "
                   "instead")]] CameraImuRotationCalibration
{
public:
    struct Options
    {
        // Imu sampling interval, default 200 Hz
        double imuInterval = 1 / 200.;

        // Estimate gyroscope bias or not during calibration
        bool estimateGyroBias = false;

        Options() {}
    };

    explicit CameraImuRotationCalibration(const Options& options = {});
    ~CameraImuRotationCalibration();

    /// Properties
    const Options& options() const;
    Options& rOptions();

    bool calculate(const StampedQuaterniondList& visualOrientations,
                   const StampedVector3dList& imuAngularVelocities,
                   Eigen::Matrix3d& imuToCameraRotation,
                   double& imuToCameraTimeOffset, Eigen::Vector3d& gyroBias);

    /// Results
    const Vector3dList& smoothedImuAngularVelocities() const;
    const Vector3dList& smoothedVisualAngularVelocities() const;

private:
    class Impl;
    const std::unique_ptr<Impl> d;

    friend class Impl;
};

} // namespace tl
