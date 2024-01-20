#pragma once

#include <sophus/se3.hpp>

#include <tMotion/ImuData>
#include <tMotion/ImuIntrinsics>
#include <tMvs/Scene>

namespace tl {

// TODO: Cleanup here, too messy!!!
class CameraImuCalibration
{
public:
    struct Options
    {
        // Initial estimation of rolling shutter camera line delay.
        // Default no delay.
        double initialRSLineDelay = 0.;

        // Calibrate rolling shutter camera line delay or not.
        bool calibrateRSLineDelay = false;

        Options() {}
    };

    explicit CameraImuCalibration(const Options& options = {});
    ~CameraImuCalibration();

    void setScene(Scene::Ptr scene);

    // TODO:
    // 1. Use Eigen than Sophus in interface
    // 2. Separate configs and initial data
    void initSpline(const Scene& scene, const Sophus::SE3d& T_i_c_init,
                    const SplineWeightingData& sewData,
                    double time_offset_imu_to_cam, const ImuDatas& imuDatas,
                    double initRSLineDelay,
                    const ImuIntrinsics& accl_intrinsics,
                    const ImuIntrinsics& gyro_intrinsics);

    void clearSpline();

    // Use this function if we really know the gravity direction of the
    // calibration board (e.g. flat on the ground -> [0, 0, 9.81])
    void setKnownGravityDir(const Eigen::Vector3d& gravity);

    void setCalibrateRSLineDelay(bool on = true);
    bool calibrateRSLineDelay() const;
    void setInitRSLineDelay(double delay);
    double initialRSLineDelay() const;
    double calibratedRSLineDelay() const;

    void setCameraId(CameraId id);

    double optimize(int iterations, int flags);

    /// Results
    std::vector<double> getCamTimestamps() const;
    StampedVector3dList getGyroMeasurements() const;
    StampedVector3dList getAcclMeasurements() const;
    void getIMUIntrinsics(ImuIntrinsics& accIntrinsics,
                          ImuIntrinsics& gyroIntrinsics, int64_t time = 0);

    void getScene(Scene& scene) const;

    Eigen::Vector3d acceleratorBiasAt(int64_t time) const;
    Eigen::Vector3d gyroscopeBiasAt(int64_t time) const;

    Eigen::Vector3d estimatedGravity() const;

    void imuToCameraTransform(Eigen::Quaterniond& orientation,
                              Eigen::Vector3d& translation) const;

private:
    class Impl;
    const std::unique_ptr<Impl> d;

    friend class Impl;
};

} // namespace tl
