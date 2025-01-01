#include "MonoCameraClient.h"
#include "DeviceClient_p.h"

#include <glog/logging.h>
#include <zmq.hpp>

#include "soc/types.h"
#include "soc/utils.h"
#include "zeromq/types.h"
#include "zeromq/utils.h"
#include "networkutils.h"

namespace tl {

namespace {

constexpr char kMonoCalibrationFilename[]{"/usr/factory/cam_parameters.yaml"};

MonoImage parseMonoData(zmq::message_t& msg)
{
    const auto data = msg.data<uint8_t>();

    MonoImage mono;
    soc::parseImageData(data, mono.image);
    mono.timestamp = reinterpret_cast<const soc::hb_vio_buffer_t*>(data)
                         ->img_info.time_stamp;

    return mono;
}

} // namespace

///------- MonoCameraClient::Impl starts from here
class MonoCameraClient::Impl : public DeviceClient::Impl
{
    MonoCameraClient* const q;

public:
    explicit Impl(MonoCameraClient* q);
    ~Impl();

    bool connectMonoSocket();
    bool disconnectMonoSocket();
    void startStreamMono();

    bool connectRectifiedMonoSocket();

    bool connectMotionSocket();
    bool disconnectMotionSocket();
    void startStreamMotion();

public:
    zmq::context_t _ctx;
    std::unique_ptr<zmq::socket_t> _monoSocket;
    std::unique_ptr<zmq::socket_t> _rectifiedMonoSocket;
    std::unique_ptr<zmq::socket_t> _motionSocket;
    std::thread _monoStreamingThread;
    std::thread _motionStreamingThread;
    std::shared_ptr<Channel<MonoImage>> _monoChannel;
    std::shared_ptr<Channel<ImuData>> _motionChannel;
    std::atomic<bool> _monoStreaming;
    std::atomic<bool> _motionStreaming;
};

MonoCameraClient::Impl::Impl(MonoCameraClient* _q)
    : DeviceClient::Impl(), q(_q), _ctx(1)
{
}

MonoCameraClient::Impl::~Impl() { q->stopStreamingMonoData(); }

bool MonoCameraClient::Impl::connectMonoSocket()
{
    _monoSocket = std::make_unique<zmq::socket_t>(_ctx, zmq::socket_type::sub);

    const auto addr = network::makeTcpAddress(_hostAddr, zeromq::kMonoDataPort);

    try {
        _monoSocket->connect(addr);
    }
    catch (const zmq::error_t& err) {
        LOG(WARNING) << "Failed to connect to mono socket: " << err.what();
        return false;
    }

    _monoSocket->set(zmq::sockopt::subscribe, "");
    _monoSocket->set(zmq::sockopt::rcvtimeo, 1500);

    LOG(INFO) << "Connect to mono socket.";
    return true;
}

bool MonoCameraClient::Impl::disconnectMonoSocket()
{
    const auto addr = network::makeTcpAddress(_hostAddr, zeromq::kMonoDataPort);

    try {
        _monoSocket->disconnect(addr);
    }
    catch (const zmq::error_t& err) {
        LOG(WARNING) << "Failed to disconnect from mono socket: " << err.what();
        return false;
    }

    LOG(INFO) << "Disconnect from mono socket.";
    return true;
}

void MonoCameraClient::Impl::startStreamMono()
{
    while (_monoStreaming.load()) {
        // image meta + image data, where meta is hb_vio_buffer_t
        zmq::message_t msg;
        const auto msgSize = _monoSocket->recv(msg, zmq::recv_flags::none);
        if (!msgSize || msgSize.value() < soc::kMinMonoDataSize) {
            continue;
        }

        if (!_monoChannel->closed()) {
            *_monoChannel << parseMonoData(msg);
        }
    }
}

bool MonoCameraClient::Impl::connectRectifiedMonoSocket()
{
    _rectifiedMonoSocket =
        std::make_unique<zmq::socket_t>(_ctx, zmq::socket_type::sub);

    const auto addr =
        network::makeTcpAddress(_hostAddr, zeromq::kRectifiedMonoDataPort);

    try {
        _rectifiedMonoSocket->connect(addr);
    }
    catch (const zmq::error_t& err) {
        LOG(WARNING) << "Failed to connect to rectified mono socket: "
                     << err.what();
        return false;
    }

    _rectifiedMonoSocket->set(zmq::sockopt::subscribe, "");
    _rectifiedMonoSocket->set(zmq::sockopt::rcvtimeo, 1500);

    LOG(INFO) << "Connect to rectified mono socket.";
    return true;
}

bool MonoCameraClient::Impl::connectMotionSocket()
{
    _motionSocket =
        std::make_unique<zmq::socket_t>(_ctx, zmq::socket_type::sub);

    const auto addr =
        network::makeTcpAddress(_hostAddr, zeromq::kMotionDataPort);

    try {
        _motionSocket->connect(addr);
    }
    catch (const zmq::error_t& err) {
        LOG(WARNING) << "Failed to connect to motion sensor socket: "
                     << err.what();
        return false;
    }

    _motionSocket->set(zmq::sockopt::subscribe, "");
    _motionSocket->set(zmq::sockopt::rcvtimeo, 1000);

    LOG(INFO) << "Connect to motion sensor socket.";
    return true;
}

bool MonoCameraClient::Impl::disconnectMotionSocket()
{
    const auto addr =
        network::makeTcpAddress(_hostAddr, zeromq::kMotionDataPort);

    try {
        _motionSocket->disconnect(addr);
    }
    catch (const zmq::error_t& err) {
        LOG(WARNING) << "Failed to disconnect from motion sensor socket: "
                     << err.what();
        return false;
    }

    LOG(INFO) << "Disconnect from motion sensor socket.";
    return true;
}

void MonoCameraClient::Impl::startStreamMotion()
{
    while (_motionStreaming.load()) {
        zmq::message_t msg;
        const auto msgSize = _motionSocket->recv(msg, zmq::recv_flags::none);
        if (!msgSize || msgSize.value() < soc::kMinMotionDataSize) {
            continue;
        }

        if (_motionChannel->closed()) {
            continue;
        }

        const auto vals = zeromq::parseMotionDatas(msg, msgSize.value());
        for (const auto& val : vals) {
            *_motionChannel << val;
        }
    }
}

///------- MonoCameraClient starts from here
MonoCameraClient::MonoCameraClient()
    : DeviceClient(*new MonoCameraClient::Impl(this))
{
}

MonoCameraClient::~MonoCameraClient() = default;

void MonoCameraClient::bindMonoDataChannel(
    std::shared_ptr<Channel<MonoImage>> channel)
{
    const auto _d = static_cast<MonoCameraClient::Impl*>(d.get());

    if (_d->_monoStreaming.load()) {
        LOG(WARNING) << "Not allowed to rebind data channel during streaming.";
        return;
    }

    if (channel) {
        _d->_monoChannel = channel;
    }
}

void MonoCameraClient::startStreamingMonoData()
{
    const auto _d = static_cast<MonoCameraClient::Impl*>(d.get());

    if (!_d->_monoChannel) {
        LOG(WARNING) << "Not allowed to stream mono data without bound "
                        "buffer channel.";
        return;
    }

    if (_d->_monoStreaming.load()) {
        LOG(WARNING) << "Mono data streaming on " << d->_hostAddr
                     << " is ongoing.";
        return;
    }

    // NOTE: Create no matter exist or not to get a clean socket
    if (!_d->connectMonoSocket()) {
        return;
    }

    _d->_monoStreaming.store(true);
    _d->_monoStreamingThread =
        std::thread{&MonoCameraClient::Impl::startStreamMono, _d};
}

void MonoCameraClient::stopStreamingMonoData()
{
    const auto _d = static_cast<MonoCameraClient::Impl*>(d.get());

    if (!_d->_monoStreaming.load()) {
        return;
    }

    _d->_monoStreaming.store(false);
    if (_d->_monoStreamingThread.joinable()) {
        _d->_monoStreamingThread.join();
    }

    _d->_monoSocket->close();
    LOG(INFO) << "Mono socket at "
              << network::makeTcpAddress(d->_hostAddr, zeromq::kMonoDataPort)
              << " closed.";
}

MonoImage MonoCameraClient::snapMonoData()
{
    const auto _d = static_cast<MonoCameraClient::Impl*>(d.get());

    if (!_d->_monoSocket || !_d->_monoSocket->handle()) {
        LOG(INFO) << "Socket is not ready for mono data. Try to reconnect...";
        if (!_d->connectMonoSocket()) {
            LOG(WARNING) << "Failed to reconnect mono data socket.";
            return {};
        }
    }

    zmq::message_t msg;
    const auto msgSize = _d->_monoSocket->recv(msg, zmq::recv_flags::none);
    if (!msgSize || msgSize.value() < soc::kMinMonoDataSize) {
        return {};
    }

    _d->_monoSocket->close();

    return parseMonoData(msg);
}

MonoImage MonoCameraClient::snapRectifiedMonoData()
{
    const auto _d = static_cast<MonoCameraClient::Impl*>(d.get());

    if (!_d->_rectifiedMonoSocket || !_d->_rectifiedMonoSocket->handle()) {
        LOG(INFO) << "Socket is not ready for rectified mono data. Try to "
                     "reconnect...";
        if (!_d->connectRectifiedMonoSocket()) {
            LOG(WARNING) << "Failed to reconnect rectified mono data socket.";
            return {};
        }
    }

    zmq::message_t msg;
    const auto msgSize =
        _d->_rectifiedMonoSocket->recv(msg, zmq::recv_flags::none);
    if (!msgSize || msgSize.value() < soc::kMinMonoDataSize) {
        return {};
    }

    _d->_rectifiedMonoSocket->close();

    return parseMonoData(msg);
}

void MonoCameraClient::bindMotionDataChannel(
    std::shared_ptr<Channel<ImuData>> channel)
{
    const auto _d = static_cast<MonoCameraClient::Impl*>(d.get());

    if (_d->_motionStreaming.load()) {
        LOG(WARNING)
            << "Not allowed to rebind motion data channel during streaming.";
        return;
    }

    if (channel) {
        _d->_motionChannel = channel;
    }
}

void MonoCameraClient::startStreamingMotionData()
{
    const auto _d = static_cast<MonoCameraClient::Impl*>(d.get());

    if (!_d->_motionChannel) {
        LOG(WARNING) << "Not allowed to stream motion data without bound "
                        "buffer channel.";
        return;
    }

    if (_d->_motionStreaming.load()) {
        LOG(WARNING) << "Motion data streaming on " << d->_hostAddr
                     << " is ongoing.";
        return;
    }

    // Create IMU streaming socket
    if (!_d->connectMotionSocket()) {
        return;
    }

    // Start IMU streaming thread
    _d->_motionStreaming.store(true);
    _d->_motionStreamingThread =
        std::thread{&MonoCameraClient::Impl::startStreamMotion, _d};
}

void MonoCameraClient::stopStreamingMotionData()
{
    const auto _d = static_cast<MonoCameraClient::Impl*>(d.get());

    if (!_d->_motionStreaming.load()) {
        return;
    }

    _d->_motionStreaming.store(false);
    if (_d->_motionStreamingThread.joinable()) {
        _d->_motionStreamingThread.join();
    }

    _d->_motionSocket->close();
    LOG(INFO) << "Motion socket at "
              << network::makeTcpAddress(d->_hostAddr, zeromq::kMotionDataPort)
              << " closed.";
}

ImuData MonoCameraClient::snapMotionData()
{
    const auto _d = static_cast<MonoCameraClient::Impl*>(d.get());

    if (!_d->_motionSocket->handle()) {
        LOG(INFO) << "Socket is not ready for motion data. Try to reconnect...";
        if (!_d->connectMotionSocket()) {
            LOG(WARNING) << "Failed to reconnect motion data socket.";
            return {};
        }
    }

    zmq::message_t msg;
    const auto msgSize = _d->_motionSocket->recv(msg, zmq::recv_flags::none);
    if (!msgSize || msgSize.value() < soc::kMinMotionDataSize) {
        return {};
    }

    return zeromq::parseMotionData(msg);
}

QCoro::Task<bool> MonoCameraClient::sendCalibrationResult(
    const std::string& res)
{
    return d->sendFile(res, kMonoCalibrationFilename);
}

QCoro::Task<std::string> MonoCameraClient::retrieveCalibrationResult()
{
    return d->retrieveFile(kMonoCalibrationFilename);
}

} // namespace tl
