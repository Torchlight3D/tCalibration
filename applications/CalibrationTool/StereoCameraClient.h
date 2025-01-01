#pragma once

#include <tCalib/CalibrationData>
#include <tCore/Channel>
#include <tMotion/ImuData>

#include "DeviceClient.h"

namespace tl {

class StereoCameraClient final : public DeviceClient
{
public:
    StereoCameraClient();
    ~StereoCameraClient();

    void bindStereoDataChannel(std::shared_ptr<Channel<StereoImages>> channel);
    void startStreamingStereoData();
    void stopStreamingStereoData();
    StereoImages snapStereoData();
    StereoImages snapRectifiedStereoData();

    void bindMotionDataChannel(std::shared_ptr<Channel<ImuData>> channel);
    void startStreamingMotionData();
    void stopStreamingMotionData();
    ImuData snapMotionData();

    QCoro::Task<bool> sendCalibrationResult(const std::string& res) override;
    QCoro::Task<std::string> retrieveCalibrationResult() override;

private:
    class Impl;
};

} // namespace tl
