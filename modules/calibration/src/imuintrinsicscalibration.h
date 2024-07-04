#pragma once

#include <memory>

#include <tMotion/ImuData>
#include <tMotion/ImuIntrinsics>

namespace tl {

// Brief:
// Imu intrinsics (misalignment, scale factors, and biases) calibration by
// sampling multiple static periods.
//
// Reference:
// imu_tk
class ImuIntrinsicsCalibration
{
public:
    struct Options
    {
        // Magnitude of measured location gracity.
        double gravity = 9.81;

        // Initial static interval, in second.
        double initStaticDuration = 30.;

        // Fixed data period used in gyroscope integration. a.k.a. dt. If the
        // value is less than 0, gyroscope timestamps are used instead. Unit???
        double gyroDataPeriod = -1.;

        // Minimum data samples to be extracted from each detected static
        // intervals.
        int minIntervalSampleCount = 100;

        // Minimum static intervals use to calibrate imu.
        int minStaticIntervalCount = 12;

        // If true, the accelerometer calibration is calculated from the mean
        // accelerations of each static interval instead of all samples.
        bool useMeanAcc = false;

        // If true, the gyroscope biases are estimated during calibration. Or
        // they are assumed known.
        bool optimizeGyroBias = false;

        Options() {}
    };

    explicit ImuIntrinsicsCalibration(const Options& options = {});
    ~ImuIntrinsicsCalibration();

    /// Options
    const Options& options() const;
    Options& rOptions();

    void setInitAcclIntrinsics(const ImuIntrinsics& intrinsics);
    const ImuIntrinsics& initAcclIntrinsics() const;
    void setInitGyroIntrinsics(const ImuIntrinsics& intrinsics);
    const ImuIntrinsics& initGyroIntrinsics() const;

    /// Calibrate
    bool calibAccelerometer(const AccDatas& samples);
    bool calibAccelerometerGyroscope(const AccDatas& accSamples,
                                     const GyroDatas& gyroSamples);

    /// Results
    const ImuIntrinsics& acclIntrinsics() const;
    const ImuIntrinsics& gyroIntrinsics() const;

    const AccDatas& calibratedAcclSamples() const;
    const GyroDatas& calibratedGyroSamples() const;

    /// IO
    bool setFromJson(const std::string& json);
    void toJsonString(std::string& json) const;

private:
    class Impl;
    const std::unique_ptr<Impl> d;

    friend class Impl;
};

} // namespace tl
