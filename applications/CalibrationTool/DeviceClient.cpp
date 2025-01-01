#include "DeviceClient.h"
#include "DeviceClient_p.h"

#include <filesystem>

// NOTE: asio is another option
#include <QUdpSocket>

#include "msgbus/types.h"
#include "networkutils.h"

#include "SharedUdpSocket.h"

namespace tl {

namespace fs = std::filesystem;

///------- DeviceClient::Impl starts from here
DeviceClient::Impl::Impl() : _localHost(std::make_unique<SharedUdpSocket>()) {}

DeviceClient::Impl::~Impl() = default;

bool DeviceClient::Impl::bindLocalAddress(const std::string& addr, int port)
{
    if (!network::isValidPort(port) || !network::isValidIPv4Address(addr)) {
        return false;
    }

    if (!_localHost) {
        _localHost = std::make_shared<SharedUdpSocket>();
    }

    // qWarning() << "Failed to bind local address at " << _localAddr << ":"
    //            << _localPort << ": " << udp.errorString();

    return _localHost->bindTo(addr, port);
}

QCoro::Task<bool> DeviceClient::Impl::setProductionModeEnable(bool on)
{
    if (!_localHost->bound()) {
        qWarning() << "Local host is not bound.";
        co_return false;
    }

    // 1. Retrieve current device working mode
    {
        msgbus::SysCmdRequest req{14, 15};
        req.setCommand(83, 2);

        const auto reply = co_await _localHost->sendRequest(
            req.toDatagram(), _hostAddr, msgbus::kDefaultPort);
        if (reply.isEmpty()) {
            qWarning() << "Request device working mode error: ";
            co_return false;
        }

        msgbus::StringResponse resp;
        resp.setFromDatagram(reply);

        if (resp.data == "pdt") {
            qInfo() << "Device is already in Production mode.";
            co_return true;
        }
    }

    // 2. Switch to Production mode
    {
        msgbus::SysCmdRequest req{14, 15};
        req.setCommand(83, 1);

        const auto reply = co_await _localHost->sendRequest(
            req.toDatagram(), _hostAddr, msgbus::kDefaultPort);
        if (reply.isEmpty()) {
            qWarning() << "Request switch to production mode error: ";
            co_return false;
        }

        msgbus::StringResponse resp;
        resp.setFromDatagram(reply);

        if (resp.data != "OK") {
            qWarning() << "Failed to switch to production mode: " << resp.data;
            co_return false;
        }
    }

    co_return true;
}

bool DeviceClient::Impl::sendCommand(const std::string& cmd)
{
    // ...
    return false;
}

QCoro::Task<bool> DeviceClient::Impl::sendFile(const std::string& content,
                                               const std::string& filename)
{
    const fs::path filePath{filename};
    // NOTE: Not sure if this validation is enough, the intention is to check
    // if the path points to a file, no matter it exists or not.
    if (!filePath.has_filename()) {
        co_return false;
    }

    if (!_localHost->bound()) {
        qWarning() << "Local host is not bound.";
        co_return false;
    }

    // 1. Enable write permission of parent directory
    qDebug() << "Try to enable directory write permission.";
    {
        msgbus::SysCmdRequest req{14, 13};
        req.setCommand(128, 8);
        req.data = std::format("mount -o remount, rw {}",
                               filePath.parent_path().string());

        const auto reply = co_await _localHost->sendRequest(
            req.toDatagram(), _hostAddr, msgbus::kDefaultPort);
        if (reply.isEmpty()) {
            qWarning() << "Request write permission error: ";
            co_return false;
        }

        msgbus::SysCmdResponse resp;
        resp.setFromDatagram(reply);

        if (resp.ret == -1) {
            qWarning() << "Failed to enable write permission: " << resp.output;
            co_return false;
        }
    }

    // 2. Enable file write mode
    qDebug() << "Try to enable file write mode.";
    {
        msgbus::FileControlRequest req{14, 13};
        req.setCommand(128, 6);
        req.mode = 1;
        req.path = filename;

        const auto reply = co_await _localHost->sendRequest(
            req.toDatagram(), _hostAddr, msgbus::kDefaultPort);

        if (reply.isEmpty()) {
            qWarning() << "Request file write mode error: ";
            co_return false;
        }

        msgbus::FileControlResponse resp;
        resp.setFromDatagram(reply);

        if (resp.err != 0) {
            qWarning() << "Failed to enable file write mode: " << resp.data;
            co_return false;
        }
    }

    // 3. Start write file
    qDebug() << "Start writing content to file.";
    {
        const auto numPages = content.size() / msgbus::kPageSize + 1;

        qDebug() << "Pending message string size: " << content.size()
                 << ", number of pages: " << numPages;

        for (auto i{0}; i < numPages; ++i) {
            msgbus::FileDataRequest req{14, 13};
            req.setCommand(128, 7);
            req.seek = i;
            req.size = msgbus::kPageSize;
            req.data =
                content.substr(i, std::min((i + 1) * msgbus::kPageSize,
                                           static_cast<int>(content.size())));

            const auto reply = co_await _localHost->sendRequest(
                req.toDatagram(), _hostAddr, msgbus::kDefaultPort);

            if (reply.isEmpty()) {
                qWarning() << "Request write file at page " << i << " error: ";
                co_return false;
            }

            msgbus::FileDataResponse resp;
            resp.setFromDatagram(reply);

            if (resp.err != 0) {
                qWarning() << "Failed to write file at page " << i << ": "
                           << resp.data;
                co_return false;
            }
        }
    }

    // 4. Close file
    qDebug() << "Closing file.";
    {
        msgbus::FileControlRequest req{14, 13};
        req.setCommand(128, 6);
        req.mode = 0;
        req.path = filename;

        const auto reply = co_await _localHost->sendRequest(
            req.toDatagram(), _hostAddr, msgbus::kDefaultPort);

        if (reply.isEmpty()) {
            qWarning() << "Request close file error: ";
            co_return false;
        }

        msgbus::FileControlResponse resp;
        resp.setFromDatagram(reply);

        if (resp.err != 0) {
            qWarning() << "Failed to close file: " << resp.data;
            co_return false;
        }
    }

    qDebug() << "Sync...";
    {
        msgbus::SysCmdRequest req{14, 13};
        req.setCommand(128, 8);
        req.data = "sync";

        const auto reply = co_await _localHost->sendRequest(
            req.toDatagram(), _hostAddr, msgbus::kDefaultPort);

        if (reply.isEmpty()) {
            qWarning() << "Request sync error: ";
            co_return false;
        }

        msgbus::SysCmdResponse resp;
        resp.setFromDatagram(reply);

        if (resp.ret == -1) {
            qWarning() << "Failed to sync: " << resp.output;
            co_return false;
        }
    }

    co_return true;
}

QCoro::Task<std::string> DeviceClient::Impl::retrieveFile(
    const std::string& filename)
{
    if (!_localHost->bound()) {
        qWarning() << "Failed to bind local address at ";
        co_return {};
    }

    // 1. Enable file read mode
    qDebug() << "Try to enable file read mode.";
    {
        msgbus::FileControlRequest req{14, 13};
        req.setCommand(128, 6);
        req.mode = 2;
        req.path = filename;

        const auto reply = co_await _localHost->sendRequest(
            req.toDatagram(), _hostAddr, msgbus::kDefaultPort);

        if (reply.isEmpty()) {
            qWarning() << "Request file read mode error: ";
            co_return {};
        }

        msgbus::FileControlResponse resp;
        resp.setFromDatagram(reply);

        if (resp.err != 0) {
            qWarning() << "Failed to enable file read mode: " << resp.data;
            co_return {};
        }
    }

    // 2. Read data
    qDebug() << "Start reading file.";
    std::string res;
    {
        auto seek{0};
        auto meetEOF{false};
        while (!meetEOF) {
            msgbus::FileDataRequest req{14, 13};
            req.setCommand(128, 7);
            req.seek = seek;
            req.size = msgbus::kPageSize;

            const auto reply = co_await _localHost->sendRequest(
                req.toDatagram(), _hostAddr, msgbus::kDefaultPort);

            if (reply.isEmpty()) {
                qWarning() << "Request read file from seek " << seek
                           << " error: ";
                co_return {};
            }

            msgbus::FileDataResponse resp;
            resp.setFromDatagram(reply);

            if (resp.err == 1) {
                qWarning() << "Read file error from seek " << seek << ": "
                           << resp.data;
                co_return {};
            }

            res.append(resp.data);

            seek += msgbus::kPageSize;
            meetEOF = (resp.err == -1);
        }
    }

    // 3. Close file
    qDebug() << "Closing file.";
    {
        msgbus::FileControlRequest req{14, 13};
        req.setCommand(128, 6);
        req.mode = 0;
        req.path = filename;

        const auto reply = co_await _localHost->sendRequest(
            req.toDatagram(), _hostAddr, msgbus::kDefaultPort);

        if (reply.isEmpty()) {
            qWarning() << "Request close file error: ";
            co_return {};
        }

        msgbus::FileControlResponse resp;
        resp.setFromDatagram(reply);

        if (resp.err != 0) {
            qWarning() << "Failed to close file: " << resp.data;
            co_return {};
        }
    }

    co_return res;
}

///------- DeviceClient starts from here
DeviceClient::DeviceClient() : DeviceClient(*new Impl) {}

DeviceClient::DeviceClient(DeviceClient::Impl& dd) : d(&dd) {}

DeviceClient::~DeviceClient() = default;

void DeviceClient::bindSharedSocket(std::shared_ptr<SharedUdpSocket> socket)
{
    if (!socket) {
        return;
    }

    d->_localHost = socket;
}

bool DeviceClient::bindTo(const std::string& localAddr, int localPort)
{
    return d->bindLocalAddress(localAddr, localPort);
}

bool DeviceClient::connectTo(const std::string& hostAddr)
{
    if (!network::isValidIPv4Address(hostAddr)) {
        return false;
    }

    d->_hostAddr = hostAddr;
    return true;
}

bool DeviceClient::isValid() const
{
    return d->_localHost && d->_localHost->bound() && !d->_hostAddr.empty();
}

QCoro::Task<bool> DeviceClient::setProductionModeEnable(bool on)
{
    return d->setProductionModeEnable(on);
}

QCoro::Task<bool> DeviceClient::genGdc()
{
    if (!d->_localHost->bound()) {
        qWarning() << "Failed to bind local address at ";
        co_return false;
    }

    // 1. Run cmd
    {
        msgbus::SysCmdRequest req{14, 13};
        req.setCommand(128, 8);
        req.data = "/app/pkgs/gdc_convert_tool/gengdc.sh";

        const auto reply = co_await d->_localHost->sendRequest(
            req.toDatagram(), d->_hostAddr, msgbus::kDefaultPort);

        if (reply.isEmpty()) {
            qWarning() << "Request gengdc error: ";
            co_return false;
        }

        msgbus::SysCmdResponse resp;
        resp.setFromDatagram(reply);

        if (resp.ret == -1) {
            qWarning() << "Failed to gengdc: " << resp.output;
            co_return false;
        }
    }

    co_return true;
}

} // namespace tl
