#include "StereoCameraClient.h"
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

constexpr char kStereoCalibrationFilename[]{"/usr/factory/cam_parameters.yaml"};

StereoImages parseStereoData(zmq::message_t& msg)
{
    // NOTE: The real message size is not exactly the same as the size
    // calculated from the message stuct.
    constexpr size_t kMagicMsgSize = 926000;

    const auto data = msg.data<uint8_t>();

    StereoImages stereo;
    soc::parseImageData(data, stereo.left);
    soc::parseImageData(data + kMagicMsgSize / 2, stereo.right);
    stereo.timestamp = reinterpret_cast<const soc::hb_vio_buffer_t*>(data)
                           ->img_info.time_stamp;

    return stereo;
}

StereoImages parseRectifiedStereoData(zmq::message_t& msg)
{
    const auto data = msg.data<uint8_t>();

    StereoImages stereo;
    const auto bufferSize = soc::parseImageData(data, stereo.left);
    soc::parseImageData(data + bufferSize, stereo.right);
    stereo.timestamp = reinterpret_cast<const soc::hb_vio_buffer_t*>(data)
                           ->img_info.time_stamp;

    return stereo;
}

} // namespace

///------- StereoCameraClient::Impl starts from here
class StereoCameraClient::Impl : public DeviceClient::Impl
{
    StereoCameraClient* const q;

public:
    explicit Impl(StereoCameraClient* q);
    ~Impl();

    bool connectStereoSocket();
    bool disconnectStereoSocket();
    void startStreamStereo();

    bool connectRectifiedStereoSocket();

    bool connectMotionSocket();
    bool disconnectMotionSocket();
    void startStreamMotion();

public:
    zmq::context_t _ctx;
    std::unique_ptr<zmq::socket_t> _stereoSocket;
    std::unique_ptr<zmq::socket_t> _rectifiedStereoSocket;
    std::unique_ptr<zmq::socket_t> _motionSocket;
    std::thread _stereoStreamingThread;
    std::thread _motionStreamingThread;
    std::shared_ptr<Channel<StereoImages>> _stereoChannel;
    std::shared_ptr<Channel<ImuData>> _motionChannel;
    std::atomic<bool> _stereoStreaming;
    std::atomic<bool> _motionStreaming;
};

StereoCameraClient::Impl::Impl(StereoCameraClient* _q)
    : DeviceClient::Impl(), q(_q), _ctx(1)
{
}

StereoCameraClient::Impl::~Impl()
{
    q->stopStreamingStereoData();
    q->stopStreamingMotionData();
}

bool StereoCameraClient::Impl::connectStereoSocket()
{
    _stereoSocket =
        std::make_unique<zmq::socket_t>(_ctx, zmq::socket_type::sub);

    const auto addr =
        network::makeTcpAddress(_hostAddr, zeromq::kStereoDataPort);

    try {
        _stereoSocket->connect(addr);
    }
    catch (const zmq::error_t& err) {
        LOG(WARNING) << "Failed to connect to stereo socket: " << err.what();
        return false;
    }

    _stereoSocket->set(zmq::sockopt::subscribe, "");
    _stereoSocket->set(zmq::sockopt::rcvtimeo, 1500);

    LOG(INFO) << "Connect to stereo socket.";
    return true;
}

bool StereoCameraClient::Impl::disconnectStereoSocket()
{
    const auto addr =
        network::makeTcpAddress(_hostAddr, zeromq::kStereoDataPort);

    try {
        _stereoSocket->disconnect(addr);
    }
    catch (const zmq::error_t& err) {
        LOG(WARNING) << "Failed to disconnect from stereo socket: "
                     << err.what();
        return false;
    }

    LOG(INFO) << "Disconnect from stereo socket.";
    return true;
}

void StereoCameraClient::Impl::startStreamStereo()
{
    while (_stereoStreaming.load()) {
        // Left meta + left image data + right mata + right image data,
        // where meta is hb_vio_buffer_t
        zmq::message_t msg;
        const auto msgSize = _stereoSocket->recv(msg, zmq::recv_flags::none);
        if (!msgSize || msgSize.value() < soc::kMinStereoDataSize) {
            continue;
        }

        if (_stereoChannel->closed()) {
            continue;
        }

        *_stereoChannel << parseStereoData(msg);
    }
}

bool StereoCameraClient::Impl::connectRectifiedStereoSocket()
{
    _rectifiedStereoSocket =
        std::make_unique<zmq::socket_t>(_ctx, zmq::socket_type::sub);

    const auto addr =
        network::makeTcpAddress(_hostAddr, zeromq::kRectifiedStereoDataPort);

    try {
        _rectifiedStereoSocket->connect(addr);
    }
    catch (const zmq::error_t& err) {
        LOG(WARNING) << "Failed to connect to rectified stereo socket: "
                     << err.what();
        return false;
    }

    _rectifiedStereoSocket->set(zmq::sockopt::subscribe, "");
    _rectifiedStereoSocket->set(zmq::sockopt::rcvtimeo, 1500);

    LOG(INFO) << "Connect to rectified stereo socket.";
    return true;
}

bool StereoCameraClient::Impl::connectMotionSocket()
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

bool StereoCameraClient::Impl::disconnectMotionSocket()
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

void StereoCameraClient::Impl::startStreamMotion()
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

///------- StereoCameraClient starts from here
StereoCameraClient::StereoCameraClient()
    : DeviceClient(*new StereoCameraClient::Impl(this))
{
}

StereoCameraClient::~StereoCameraClient() = default;

void StereoCameraClient::bindStereoDataChannel(
    std::shared_ptr<Channel<StereoImages>> channel)
{
    const auto _d = static_cast<StereoCameraClient::Impl*>(d.get());

    if (_d->_stereoStreaming.load()) {
        LOG(WARNING) << "Not allowed to rebind data channel during streaming.";
        return;
    }

    if (channel) {
        _d->_stereoChannel = channel;
    }
}

void StereoCameraClient::startStreamingStereoData()
{
    const auto _d = static_cast<StereoCameraClient::Impl*>(d.get());

    if (!_d->_stereoChannel) {
        LOG(WARNING) << "Not allowed to stream stereo data without bound "
                        "buffer channel.";
        return;
    }

    if (_d->_stereoStreaming.load()) {
        LOG(WARNING) << "Stereo data streaming on " << d->_hostAddr
                     << " is ongoing.";
        return;
    }

    // NOTE: Create no matter exist or not to get a clean socket
    if (!_d->connectStereoSocket()) {
        return;
    }

    _d->_stereoStreaming.store(true);
    _d->_stereoStreamingThread =
        std::thread{&StereoCameraClient::Impl::startStreamStereo, _d};
}

void StereoCameraClient::stopStreamingStereoData()
{
    const auto _d = static_cast<StereoCameraClient::Impl*>(d.get());

    if (!_d->_stereoStreaming.load()) {
        return;
    }

    _d->_stereoStreaming.store(false);
    if (_d->_stereoStreamingThread.joinable()) {
        _d->_stereoStreamingThread.join();
    }

    _d->_stereoSocket->close();
    LOG(INFO) << "Stereo socket at "
              << network::makeTcpAddress(d->_hostAddr, zeromq::kStereoDataPort)
              << " closed.";
}

StereoImages StereoCameraClient::snapStereoData()
{
    const auto _d = static_cast<StereoCameraClient::Impl*>(d.get());

    if (!_d->_stereoSocket || !_d->_stereoSocket->handle()) {
        LOG(INFO) << "Socket is not ready for stereo data. Try to reconnect...";
        if (!_d->connectStereoSocket()) {
            LOG(WARNING) << "Failed to reconnect stereo data socket.";
            return {};
        }
    }

    zmq::message_t msg;
    const auto msgSize = _d->_stereoSocket->recv(msg, zmq::recv_flags::none);
    if (!msgSize || msgSize.value() < soc::kMinStereoDataSize) {
        return {};
    }

    _d->_stereoSocket->close();

    return parseStereoData(msg);
}

StereoImages StereoCameraClient::snapRectifiedStereoData()
{
    const auto _d = static_cast<StereoCameraClient::Impl*>(d.get());

    if (!_d->_rectifiedStereoSocket || !_d->_rectifiedStereoSocket->handle()) {
        LOG(INFO) << "Socket is not ready for rectified stereo data. Try to "
                     "reconnect...";
        if (!_d->connectRectifiedStereoSocket()) {
            LOG(WARNING) << "Failed to reconnect rectified stereo data socket.";
            return {};
        }
    }

    zmq::message_t msg;
    const auto msgSize =
        _d->_rectifiedStereoSocket->recv(msg, zmq::recv_flags::none);
    if (!msgSize || msgSize.value() < soc::kMinStereoDataSize) {
        return {};
    }

    _d->_rectifiedStereoSocket->close();

    return parseRectifiedStereoData(msg);
}

void StereoCameraClient::bindMotionDataChannel(
    std::shared_ptr<Channel<ImuData>> channel)
{
    const auto _d = static_cast<StereoCameraClient::Impl*>(d.get());

    if (_d->_motionStreaming.load()) {
        LOG(WARNING)
            << "Not allowed to rebind motion data channel during streaming.";
        return;
    }

    if (channel) {
        _d->_motionChannel = channel;
    }
}

void StereoCameraClient::startStreamingMotionData()
{
    const auto _d = static_cast<StereoCameraClient::Impl*>(d.get());

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
        std::thread{&StereoCameraClient::Impl::startStreamMotion, _d};
}

void StereoCameraClient::stopStreamingMotionData()
{
    const auto _d = static_cast<StereoCameraClient::Impl*>(d.get());

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

ImuData StereoCameraClient::snapMotionData()
{
    const auto _d = static_cast<StereoCameraClient::Impl*>(d.get());

    if (!_d->_motionSocket || !_d->_motionSocket->handle()) {
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

QCoro::Task<bool> StereoCameraClient::sendCalibrationResult(
    const std::string& res)
{
    return d->sendFile(res, kStereoCalibrationFilename);
}

QCoro::Task<std::string> StereoCameraClient::retrieveCalibrationResult()
{
    return d->retrieveFile(kStereoCalibrationFilename);
}

} // namespace tl
