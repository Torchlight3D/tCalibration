#pragma once

#include <memory>
#include <string>

#include <QCoroCore>

namespace tl {

inline constexpr char kDefaultLocalAddr[]{"9.0.0.100"};
inline constexpr auto kDefaultLocalMsgBusPort{9000};

class SharedUdpSocket;

// TODO:
// 1. Should we use shared zmq context?
// 2. Add callback style interface, what's the potential problem?
// 3. There are still a lot of similar code, what's the better design?
class DeviceClient
{
public:
    DeviceClient();
    virtual ~DeviceClient();

    // Bind to shared local host
    void bindSharedSocket(std::shared_ptr<SharedUdpSocket> socket);
    // Create and bind to local host
    bool bindTo(const std::string& localAddress, int localPort);

    // No actual connection takes place. The IP address is saved for later use.
    virtual bool connectTo(const std::string& hostAddress);
    // TODO: Define what is valid
    virtual bool isValid() const;

    QCoro::Task<bool> setProductionModeEnable(bool on);

    virtual QCoro::Task<bool> sendCalibrationResult(const std::string& res) = 0;
    virtual QCoro::Task<std::string> retrieveCalibrationResult() = 0;

    virtual QCoro::Task<bool> genGdc();

protected:
    class Impl;
    const std::unique_ptr<Impl> d;

protected:
    explicit DeviceClient(DeviceClient::Impl& dd);
};

} // namespace tl
