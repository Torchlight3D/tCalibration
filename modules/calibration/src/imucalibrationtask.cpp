#include "imucalibrationtask.h"

#include <tCalibration/ImuIntrinsicsCalibration>
#include <tMotion/ImuData>

namespace tl {

bool loadData(const std::string& filePath)
{
    // TODO
    return false;
}

void calibrationImuIntrinsics()
{
    //
}

//
void estimateImuNoiseParameters()
{
    // TODO: Change to real loaded data
    // ...
    ImuDatas datas;

    ImuIntrinsicsCalibration calibration{};
    auto& opts = calibration.rOptions();
    opts.initStaticDuration = 120;
    opts.minStaticIntervalCount = 12;
    opts.minIntervalSampleCount = 100; // ~ how many equivalent sec

    calibration.calibAccelerometerGyroscope(datas.acc, datas.gyro);

    // TODO: Write configs and results
    std::string configs;
    calibration.toJsonString(configs);

    const auto& acclIntrinsics = calibration.acclIntrinsics();
    const auto& gyroIntrinsics = calibration.gyroIntrinsics();
    // ...
}

bool saveResult(const std::string& filePath)
{
    //
    return false;
}

} // namespace tl
