#pragma once

#include <QCoroCore>

#include "DeviceClient.h"

namespace tl {

class SharedUdpSocket;

class DeviceClient::Impl
{
public:
    Impl();
    virtual ~Impl();

    bool bindLocalAddress(const std::string& address, int port);

    QCoro::Task<bool> setProductionModeEnable(bool on);

    // NOTE: Not marked as const.
    // For many network-related interfaces, although they may appear to be
    // const, the underlying handle (or the buffer within the handle) changes
    // with each data transmission.

    bool sendCommand(const std::string& cmd);

    QCoro::Task<bool> sendFile(const std::string& content,
                               const std::string& filename);
    QCoro::Task<std::string> retrieveFile(const std::string& filename);

public:
    std::shared_ptr<SharedUdpSocket> _localHost;
    std::string _hostAddr;
};

} // namespace tl
