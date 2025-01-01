#pragma once

#include <tCalib/CalibrationData>
#include <tCore/Channel>
#include <tMotion/ImuData>

#include "DeviceClient.h"

namespace tl {

class MonoCameraClient final : public DeviceClient
{
public:
    MonoCameraClient();
    ~MonoCameraClient();

    void bindMonoDataChannel(std::shared_ptr<Channel<MonoImage>> channel);
    void startStreamingMonoData();
    void stopStreamingMonoData();
    MonoImage snapMonoData();
    MonoImage snapRectifiedMonoData();

    // FIXME: Duplicated in StereoCameraClient.
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
