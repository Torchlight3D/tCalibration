#include "StereoModuleHubClient.h"

#include <format>

#include <grpcpp/alarm.h>
#include <grpcpp/grpcpp.h>
#include <opencv2/imgcodecs.hpp>

#include <QtConcurrent/QtConcurrent>

#include <tCore/ContainerUtils>

namespace tl {

namespace {
const int stereoDataType = qRegisterMetaType<const StereoImageData&>();

inline StereoModuleInfo toStereoModuleInfo(const pb::DeviceInfo& pbInfo)
{
    StereoModuleInfo info;
    info.maskedIp = pbInfo.name();
    info.hardwareAddr = pbInfo.hw_addr();
    info.connected = pbInfo.status();
    return info;
};

inline cv::Mat bytesToCvMat(const std::string& bytes)
{
    return cv::imdecode(std::vector<uchar>{bytes.begin(), bytes.end()},
                        cv::IMREAD_COLOR);
}

inline cv::Mat bytesToGray(const std::string& bytes)
{
    // ATTENTION:
    // It takes really long time to trace the bug here, if we copy the data,
    // streaming becomes unstable.
    //    std::string data = bytes;
    return {480, 640, CV_8UC1, const_cast<char*>(bytes.data())};
}

inline ImuData pbImuDataToImuData(const pb::ImuData& imu)
{
    return {.acc = {imu.timestamp(), imu.acc_x(), imu.acc_y(), imu.acc_z()},
            .gyro = {imu.timestamp(), imu.gyr_x(), imu.gyr_y(), imu.gyr_z()},
            .temperature = imu.temp()};
};

inline std::vector<ImuData> pbImuDataListToImuDataList(
    const pb::ImuDataList& data)
{
    std::vector<ImuData> ret;
    ret.reserve(data.data_size());
    for (int i{0}; i < data.data_size(); ++i) {
        ret.push_back(pbImuDataToImuData(data.data(i)));
    }
    return ret;
}

} // namespace

/// Async style request
///------- AsyncRequest starts from here
// class AsyncHandle
//{
// public:
//     enum Operation
//     {
//         Connect,
//         Read,
//         Write,
//         WriteFinished,
//         Finished,
//     };

//    AsyncHandle(AsyncRequest& instace, Operation op)
//        : instance_(instace), op_(op)
//    {
//    }

//    [[nodiscard]] void* tag()
//    {
//        ++instance_.ref_cnt_;
//        return this;
//    }

//    void proceed(bool ok)
//    {
//        assert(instance_.m_refCount > 0);
//        --instance_.ref_cnt_;

//        constexpr bool do_push_back_on_queue{true};
//        if (do_push_back_on_queue) {
//            // Handle failures immediately.
//            if (ok && !pushed_back_) {
//                // Work-around to push the event to the end of the queue.
//                // By default the "queue" works like a stack, which is not
//                // what most devs excpect or want. Ref:
//                //
//                https://www.gresearch.com/blog/article/lessons-learnt-from-writing-asynchronous-streaming-grpc-services-in-c/
//                alarm_.Set(&instance_.m_owner.cq_,
//                           gpr_now(gpr_clock_type::GPR_CLOCK_REALTIME),
//                           tag());
//                pushed_back_ = true;
//                pushed_ok_ = ok;

//                return;
//            }

//            if (pushed_back_) {
//                ok = pushed_ok_;

//                // Now we are ready for the next operation on this tag.
//                pushed_back_ = false;
//            }
//        }

//        instance_.proceed(ok, op_);

//        if (instance_.ref_cnt_ == 0) {
//            instance_.done();
//        }
//    }

// private:
//     AsyncRequest& instance_;
//     const Operation op_;
//     bool pushed_back_ = false;
//     bool pushed_ok_ = false;
//     ::grpc::Alarm alarm_;
// };

// NOTE:
// 1. We make AsyncRequest a QObject to use signal&slot mechanism to pass
// received data to host.
// 2. We dont use QObject tree to manage AsyncRequest live time.
class AsyncRequest : public QObject
{
    Q_OBJECT
public:
    // Tag
    class Handle
    {
    public:
        enum Operation
        {
            Connect,
            Read,
            Write,
            WriteFinished,
            Finished,
        };

        Handle(AsyncRequest& instace, Operation op)
            : instance_(instace), op_(op)
        {
        }

        [[nodiscard]] void* tag()
        {
            ++instance_.ref_cnt_;
            return this;
        }

        void proceed(bool ok)
        {
            assert(instance_.ref_cnt_ > 0);
            --instance_.ref_cnt_;

            constexpr bool do_push_back_on_queue{true};
            if (do_push_back_on_queue) {
                // Handle failures immediately.
                if (ok && !pushed_back_) {
                    // Work-around to push the event to the end of the queue.
                    // By default the "queue" works like a stack, which is not
                    // what most devs excpect or want. Ref:
                    // https://www.gresearch.com/blog/article/lessons-learnt-from-writing-asynchronous-streaming-grpc-services-in-c/
                    alarm_.Set(&instance_.m_owner.m_streamQueue,
                               gpr_now(gpr_clock_type::GPR_CLOCK_REALTIME),
                               tag());
                    pushed_back_ = true;
                    pushed_ok_ = ok;

                    return;
                }

                if (pushed_back_) {
                    ok = pushed_ok_;

                    // Now we are ready for the next operation on this tag.
                    pushed_back_ = false;
                }
            }

            instance_.proceed(ok, op_);

            if (instance_.ref_cnt_ == 0) {
                instance_.done();
            }
        }

    private:
        AsyncRequest& instance_;
        const Operation op_;
        bool pushed_back_ = false;
        bool pushed_ok_ = false;
        ::grpc::Alarm alarm_;
    };

    AsyncRequest(StereoModuleHubClient& host)
        : QObject(nullptr), m_owner(host), client_id_(++host.next_client_id_)
    {
        LOG(INFO) << "Constructed request #" << client_id_ << " at address"
                  << this;
    }

    virtual ~AsyncRequest() = default;

    // The state-machine
    virtual void proceed(bool ok, Handle::Operation op) = 0;

protected:
    StereoModuleHubClient& m_owner;
    ::grpc::ClientContext ctx_;
    const size_t client_id_;
    int ref_cnt_ = 0;

private:
    void done()
    {
        // TODO
        // Delete reference counting from parent, and delete self
        m_owner.decCounter();
        delete this;
    }
};

class GetDeviceInfosRequest : public AsyncRequest
{
    Q_OBJECT

public:
    explicit GetDeviceInfosRequest(StereoModuleHubClient& host);

    void proceed(bool ok, Handle::Operation op) override;

private:
    Handle handle_{*this, Handle::Operation::Finished};
    pb::Empty req_;
    pb::Devices reply_;
    ::grpc::Status status_;
    std::unique_ptr<::grpc::ClientAsyncResponseReader<pb::Devices>> reader_;
};

GetDeviceInfosRequest::GetDeviceInfosRequest(StereoModuleHubClient& host)
    : AsyncRequest(host)
{
    reader_ = m_owner.stub()->AsyncGetDevicesInfos(&ctx_, req_,
                                                   &m_owner.m_streamQueue);
    assert(reader_);

    reader_->Finish(&reply_, &status_, handle_.tag());
    m_owner.incCounter();
}

void GetDeviceInfosRequest::proceed(bool ok, Handle::Operation /*op */)
{
    if (!ok) [[unlikely]] {
        LOG(INFO) << "Failed to request current devices: "
                  << status_.error_message();
        return;
    }

    if (status_.ok()) {
        LOG(INFO) << "Current devices updated.  " << reply_.info().size();
    }
    else {
        LOG(INFO) << "Request device error: " << status_.error_message();
    }

    // The reply is a single message, so at this time we are done.
}

///------- StreamStereoImageRequest starts from here
class StreamStereoImageRequest : public AsyncRequest
{
public:
    StreamStereoImageRequest(StereoModuleHubClient& client,
                             const pb::DeviceInfo& device, int index);

    void proceed(bool ok, Handle::Operation op) override;

private:
    Handle connect_handle_{*this, Handle::Operation::Connect};
    Handle read_handle_{*this, Handle::Operation::Read};
    Handle finish_handle_{*this, Handle::Operation::Finished};

    const pb::DeviceInfo& request_;
    pb::StereoYUVData reply_;
    ::grpc::Status status_;
    std::unique_ptr<::grpc::ClientAsyncReader<pb::StereoYUVData>> reader_;
    const int index_;
};

// Now we are implementing an actual, trivial state-machine, as
// we will read an unknown number of messages.
StreamStereoImageRequest::StreamStereoImageRequest(
    StereoModuleHubClient& client, const pb::DeviceInfo& device, int index)
    : AsyncRequest(client), request_(device), index_(index)
{
    reader_ = m_owner.stub()->AsyncStartYUVStream(
        &ctx_, request_, &m_owner.m_streamQueue, connect_handle_.tag());
    assert(reader_);

    reader_->Finish(&status_, finish_handle_.tag());
    m_owner.incCounter();
}

// We have three states to deal with on each invocation:
// 1. The state of the instance - how many async operations have we started?
// This is handled by reference-counting, so we don't have to deal with it in
// the loop.
// 2. The operation
// 3. The ok boolean value.
void StreamStereoImageRequest::proceed(bool ok, Handle::Operation op)
{
    // LOG(INFO) << "Handle stream stereo data event at dock " << index_
    //          << ": status=" << ok << ", operation=" << op;

    switch (op) {
        case Handle::Operation::Connect: {
            if (!ok) [[unlikely]] {
                LOG(WARNING) << "Failed to start streaming stereo data at dock "
                             << index_;
                return;
            }

            LOG(INFO) << "Start stereo data streaming at dock " << index_;
            reader_->Read(&reply_, read_handle_.tag());
        } break;
        case Handle::Operation::Read: {
            if (!ok) [[unlikely]] {
                LOG(WARNING)
                    << "Streaming stereo data error at dock " << index_;
                return;
            }

            // Received data handling here
            // LOG(INFO) << "Received stereo image data from dock " << index_
            //           << ", at " << reply_.left_timestamp();
            emit m_owner.stereoImageDataReceived(
                {bytesToGray(reply_.left_y()), bytesToGray(reply_.right_y()),
                 reply_.left_timestamp()},
                index_);

            reply_.Clear();

            reader_->Read(&reply_, read_handle_.tag());
        } break;
        case Handle::Operation::Finished: {
            LOG(INFO) << "Request to stop stereo data streaming at dock "
                      << index_;
            if (!ok) [[unlikely]] {
                LOG(WARNING)
                    << "Failed to finish stereo data streaming at dock "
                    << index_ << ": " << status_.error_message();
                return;
            }

            if (status_.ok()) {
                LOG(INFO) << "Streaming stereo data at dock " << index_
                          << " is about to stop.";
                LOG(INFO) << "Initiating a new request at dock " << index_;
            }
            else {
                LOG(WARNING) << "Failed to stop stereo data streaming at dock "
                             << index_ << ": " << status_.error_message();
            }
        } break;
        default:
            LOG(WARNING) << "Unexpected operation in state-machine: "
                         << static_cast<int>(op);
            assert(false);
            break;
    }
}

///------- StopStreamStereoImageRequest starts from here
class StopStreamStereoImageRequest : public AsyncRequest
{
    Q_OBJECT

public:
    explicit StopStreamStereoImageRequest(StereoModuleHubClient& host,
                                          const pb::DeviceInfo& device);

    void proceed(bool ok, Handle::Operation op) override;

private:
    Handle handle_{*this, Handle::Operation::Finished};
    const pb::DeviceInfo& req_;
    pb::CaptureResult reply_;
    ::grpc::Status status_;
    std::unique_ptr<::grpc::ClientAsyncResponseReader<pb::CaptureResult>>
        reader_;
};

StopStreamStereoImageRequest::StopStreamStereoImageRequest(
    StereoModuleHubClient& host, const pb::DeviceInfo& device)
    : AsyncRequest(host), req_(device)
{
    reader_ =
        m_owner.stub()->AsyncStopYUVStream(&ctx_, req_, &m_owner.m_streamQueue);
    assert(reader_);

    reader_->Finish(&reply_, &status_, handle_.tag());
    m_owner.incCounter();
}

void StopStreamStereoImageRequest::proceed(bool ok, Handle::Operation /*op */)
{
    if (!ok) [[unlikely]] {
        LOG(INFO) << "Failed to request current devices: "
                  << status_.error_message();
        return;
    }

    if (status_.ok()) {
        LOG(INFO) << "Stop stereo image streaming: " << reply_.name();
    }
    else {
        LOG(INFO) << "Request device erro: " << status_.error_message();
    }

    // The reply is a single message, so at this time we are done.
}

///------- StreamImuDataRequest starts from here
class StreamImuDataRequest : public AsyncRequest
{
    Q_OBJECT

public:
    StreamImuDataRequest(StereoModuleHubClient& host,
                         const pb::DeviceInfo& device, int index);

    void proceed(bool ok, Handle::Operation op) override;

signals:
    void imuDataReady(const ImuData& imu);

private:
    Handle connect_handle_{*this, Handle::Operation::Connect};
    Handle read_handle_{*this, Handle::Operation::Read};
    Handle finish_handle_{*this, Handle::Operation::Finished};

    const pb::DeviceInfo& request_;
    pb::ImuDataList reply_;
    ::grpc::Status status_;
    std::unique_ptr<::grpc::ClientAsyncReader<pb::ImuDataList>> reader_;
    const int index_;
};

StreamImuDataRequest::StreamImuDataRequest(StereoModuleHubClient& client,
                                           const pb::DeviceInfo& device,
                                           int index)
    : AsyncRequest(client), request_(device), index_(index)
{
    reader_ = m_owner.stub()->AsyncStartImuDataStream(
        &ctx_, request_, &m_owner.m_streamQueue, connect_handle_.tag());
    assert(reader_);

    reader_->Finish(&status_, finish_handle_.tag());
    m_owner.incCounter();
}

void StreamImuDataRequest::proceed(bool ok, Handle::Operation op)
{
    // LOG(INFO) << "Handle stream IMU data event at dock " << index_
    //           << ": status=" << ok << ", operation=" << op;

    switch (op) {
        case Handle::Operation::Connect: {
            if (!ok) {
                LOG(WARNING)
                    << "Failed to start streaming IMU data at dock " << index_;
                return;
            }

            LOG(INFO) << "Start IMU data streaming at dock " << index_;
            reader_->Read(&reply_, read_handle_.tag());
        } break;
        case Handle::Operation::Read: {
            if (!ok) {
                LOG(WARNING) << "Streaming IMU data error at dock " << index_;
                return;
            }

            // Received data handling here
            // LOG(INFO) << "Received IMU image data from dock " << index_
            //           << ", at " << reply_.timestamp();
            emit m_owner.imuDataListReceived(pbImuDataListToImuDataList(reply_),
                                             index_);

            reply_.Clear();

            reader_->Read(&reply_, read_handle_.tag());
        } break;
        case Handle::Operation::Finished: {
            LOG(INFO) << "Request to stop IMU streaming at dock " << index_;
            if (!ok) {
                LOG(WARNING) << "Failed to finish IMU data streaming at dock "
                             << index_ << ": " << status_.error_message();
                return;
            }

            if (status_.ok()) {
                LOG(INFO) << "Streaming IMU data at dock " << index_
                          << " is about to stop.";
                LOG(INFO) << "Initiating a new request at dock " << index_;
            }
            else {
                LOG(WARNING) << "Failed to stop IMU data streaming at dock "
                             << index_ << ": " << status_.error_message();
            }
        } break;
        default:
            LOG(INFO) << "Unexpected operation in state-machine: "
                      << static_cast<int>(op);
            assert(false);
            break;
    }
}

///------- StopStreamStereoImageRequest starts from here
class StopImuStreamRequest : public AsyncRequest
{
    Q_OBJECT

public:
    explicit StopImuStreamRequest(StereoModuleHubClient& host,
                                  const pb::DeviceInfo& device);

    void proceed(bool ok, Handle::Operation op) override;

private:
    Handle handle_{*this, Handle::Operation::Finished};
    const pb::DeviceInfo& req_;
    pb::CaptureResult reply_;
    ::grpc::Status status_;
    std::unique_ptr<::grpc::ClientAsyncResponseReader<pb::CaptureResult>>
        reader_;
};

StopImuStreamRequest::StopImuStreamRequest(StereoModuleHubClient& host,
                                           const pb::DeviceInfo& device)
    : AsyncRequest(host), req_(device)
{
    reader_ = m_owner.stub()->AsyncStopImuDataStream(&ctx_, req_,
                                                     &m_owner.m_streamQueue);
    assert(reader_);

    reader_->Finish(&reply_, &status_, handle_.tag());
    m_owner.incCounter();
}

void StopImuStreamRequest::proceed(bool ok, Handle::Operation /*op */)
{
    if (!ok) [[unlikely]] {
        LOG(INFO) << "Failed to request current devices: "
                  << status_.error_message();
        return;
    }

    if (status_.ok()) {
        LOG(INFO) << "Stop imu streaming: " << reply_.name();
    }
    else {
        LOG(INFO) << "Request device erro: " << status_.error_message();
    }

    // The reply is a single message, so at this time we are done.
}

/// Callback style request
///------- CallbackRequst starts from here
class CallbackRequest
{
public:
    explicit CallbackRequest(StereoModuleHubClient& owner) : m_owner(owner)
    {
        ++m_owner.m_runningCount;
    }

    ~CallbackRequest()
    {
        if (--m_owner.m_runningCount == 0) {
            m_owner.m_doneStreaming.set_value();
        }
    }

protected:
    StereoModuleHubClient& m_owner;
};

///------- StartYUVStreamCallbackRequest starts from here
using YUVDataOrStatus = std::variant<const pb::StereoYUVData*, ::grpc::Status>;
using StartYUVStreamCallback = std::function<void(YUVDataOrStatus)>;

class StartYUVStreamCallbackRequest
    : public CallbackRequest,
      ::grpc::ClientReadReactor<pb::StereoYUVData>
{
public:
    StartYUVStreamCallbackRequest(StereoModuleHubClient& owner,
                                  const pb::DeviceInfo& request,
                                  StartYUVStreamCallback&& cb);

    void OnReadDone(bool ok) override;
    void OnDone(const ::grpc::Status& status) override;

private:
    ::grpc::ClientContext m_context;
    const pb::DeviceInfo& m_request;
    pb::StereoYUVData m_response;
    StartYUVStreamCallback m_callback;
};

StartYUVStreamCallbackRequest::StartYUVStreamCallbackRequest(
    StereoModuleHubClient& owner, const pb::DeviceInfo& request,
    StartYUVStreamCallback&& cb)
    : CallbackRequest(owner), m_request(request), m_callback(std::move(cb))
{
    LOG(INFO) << "Start stereo YUV stream request.";

    m_owner.stub()->async()->StartYUVStream(&m_context, &m_request, this);

    StartRead(&m_response);
    StartCall();
}

void StartYUVStreamCallbackRequest::OnReadDone(bool ok)
{
    if (ok) {
        // LOG(INFO) << "Received stereo data at " <<
        // m_response.left_timestamp();

        m_callback(&m_response);

        m_response.Clear();
        return StartRead(&m_response);
    }

    LOG(WARNING) << "Failed to eceived stereo data at ";
}

void StartYUVStreamCallbackRequest::OnDone(const ::grpc::Status& status)
{
    if (status.ok()) {
        LOG(INFO) << "Request succeeded.";
    }
    else {
        LOG(WARNING) << "Request failed: " << status.error_message();
    }

    m_callback(status);
    delete this;
}

///------- StartImuDataStreamCallbackRequest starts from here
using ImuDataOrStatus = std::variant<const pb::ImuDataList*, ::grpc::Status>;
using StartImuDataStreamCallback = std::function<void(ImuDataOrStatus)>;

class StartImuDataStreamCallbackRequest
    : public CallbackRequest,
      ::grpc::ClientReadReactor<pb::ImuDataList>
{
public:
    StartImuDataStreamCallbackRequest(StereoModuleHubClient& owner,
                                      const pb::DeviceInfo& request,
                                      StartImuDataStreamCallback&& cb);

    void OnReadDone(bool ok) override;
    void OnDone(const ::grpc::Status& status) override;

private:
    ::grpc::ClientContext m_context;
    const pb::DeviceInfo& m_request;
    pb::ImuDataList m_response;
    StartImuDataStreamCallback m_callback;
};

StartImuDataStreamCallbackRequest::StartImuDataStreamCallbackRequest(
    StereoModuleHubClient& owner, const pb::DeviceInfo& request,
    StartImuDataStreamCallback&& cb)
    : CallbackRequest(owner), m_request(request), m_callback(std::move(cb))
{
    LOG(INFO) << "Start IMU stream request.";

    m_owner.stub()->async()->StartImuDataStream(&m_context, &m_request, this);

    StartRead(&m_response);
    StartCall();
}

void StartImuDataStreamCallbackRequest::OnReadDone(bool ok)
{
    if (ok) {
        // LOG(INFO) << "Received IMU data at " << m_response.timestamp();

        m_callback(&m_response);

        m_response.Clear();
        return StartRead(&m_response);
    }

    LOG(WARNING) << "Failed to eceived stereo data at ";
}

void StartImuDataStreamCallbackRequest::OnDone(const ::grpc::Status& status)
{
    if (status.ok()) {
        LOG(INFO) << "Request succeeded.";
    }
    else {
        LOG(WARNING) << "Request failed: " << status.error_message();
    }

    m_callback(status);
    delete this;
}

///------- StopYUVStreamCallbackRequest starts from here
using StopYUVStreamCallback =
    std::function<void(const ::grpc::Status&, const pb::CaptureResult&)>;

class StopYUVStreamCallbackRequest : public CallbackRequest
{
public:
    StopYUVStreamCallbackRequest(StereoModuleHubClient& owner,
                                 const pb::DeviceInfo& request,
                                 StopYUVStreamCallback&& cb)
        : CallbackRequest(owner), m_request(request), m_callback(std::move(cb))
    {
        LOG(INFO) << "About to stop stereo data stream...";

        m_owner.stub()->async()->StopYUVStream(
            &m_context, &m_request, &m_response,
            [this](const ::grpc::Status& status) {
                LOG(INFO) << "getFeature calling finished callback.";
                m_callback(status, m_response);
                delete this;
            });
    }

private:
    ::grpc::ClientContext m_context;
    const pb::DeviceInfo& m_request;
    pb::CaptureResult m_response;
    StopYUVStreamCallback m_callback;
};

///------- StopImuDataStreamCallbackRequest starts from here
using StopImuDataStreamCallback =
    std::function<void(const ::grpc::Status&, const pb::CaptureResult&)>;

class StopImuDataStreamCallbackRequest : public CallbackRequest
{
public:
    StopImuDataStreamCallbackRequest(StereoModuleHubClient& owner,
                                     const pb::DeviceInfo& request,
                                     StopImuDataStreamCallback&& cb)
        : CallbackRequest(owner), m_request(request), m_callback(std::move(cb))
    {
        LOG(INFO) << "About to stop IMU data stream...";

        m_owner.stub()->async()->StopImuDataStream(
            &m_context, &m_request, &m_response,
            [this](const ::grpc::Status& status) {
                LOG(INFO) << "Finishing IMU data...";
                m_callback(status, m_response);
                delete this;
            });
    }

private:
    ::grpc::ClientContext m_context;
    const pb::DeviceInfo& m_request;
    pb::CaptureResult m_response;
    StopImuDataStreamCallback m_callback;
};

///------- StereoModuleHubClientPrivate starts from here
class StereoModuleHubClientPrivate
{
    Q_DISABLE_COPY(StereoModuleHubClientPrivate)
    Q_DECLARE_PUBLIC(StereoModuleHubClient)
    StereoModuleHubClient* const q_ptr;

public:
    explicit StereoModuleHubClientPrivate(StereoModuleHubClient* q);
    ~StereoModuleHubClientPrivate();

    void init(const StereoModuleHubClient::Configs& configs);

    void updateCurrentDevices();

    const pb::DeviceInfo& device(int index) const;
    bool deviceAvailable(int index) const;

public:
    StereoModuleHubClient::Configs m_configs;
    std::shared_ptr<::grpc::Channel> m_channel;
    std::unique_ptr<pb::CameraController::Stub> m_stub;
    pb::Devices m_cachedDeviceInfos;
    std::vector<int> m_validDeviceIndices;
    StereoImageDataStream* m_stereoImageDataStream{nullptr}; // Not owned
};

StereoModuleHubClientPrivate::StereoModuleHubClientPrivate(
    StereoModuleHubClient* q)
    : q_ptr(q)
{
}

StereoModuleHubClientPrivate::~StereoModuleHubClientPrivate()
{
    Q_Q(StereoModuleHubClient);
    // FIXME: Need to stop streaming before destroy in case GUI (or other
    // callers) forget to stop. But when the program arrives here,
    // onlineDeviceIndices becomes empty
    // q->blockStopStreaming();
}

void StereoModuleHubClientPrivate::init(
    const StereoModuleHubClient::Configs& configs)
{
    constexpr auto mbToByte = [](int size) { return size * 1024 * 1024; };

    auto createChannel = [&](const StereoModuleHubClient::Configs& configs)
        -> std::shared_ptr<::grpc::Channel> {
        ::grpc::ChannelArguments args;
        // Use SetInt(property_id) or alternative
        // GRPC_ARG_MAX_RECEIVE_MESSAGE_LENGTH
        // GRPC_ARG_MAX_SEND_MESSAGE_LENGTH
        args.SetMaxSendMessageSize(mbToByte(configs.maxSendMessageSize));
        args.SetMaxReceiveMessageSize(mbToByte(configs.maxReceivedMessageSize));

        return ::grpc::CreateCustomChannel(
            configs.hostAddress, ::grpc::InsecureChannelCredentials(), args);
    };

    m_configs = configs;
    m_channel = createChannel(m_configs);
    m_stub = std::move(pb::CameraController::NewStub(m_channel));
}

void StereoModuleHubClientPrivate::updateCurrentDevices()
{
    pb::Empty request;
    ::grpc::ClientContext context;
    const auto status =
        m_stub->GetDevicesInfos(&context, request, &m_cachedDeviceInfos);
    if (status.ok()) {
        LOG(INFO) << "Server tried to connect "
                  << m_cachedDeviceInfos.info_size() << " devices.";

        m_validDeviceIndices.clear();
        for (int i{0}; i < m_cachedDeviceInfos.info_size(); ++i) {
            const auto& info = m_cachedDeviceInfos.info(i);
            if (info.status()) {
                m_validDeviceIndices.push_back(i);
                LOG(INFO) << std::format("{0}({1}) at dock {2} is connected.",
                                         info.name(), info.hw_addr(), i);
            }
        }
    }
    else {
        LOG(WARNING) << "Failed to update devices: " << status.error_message();
    }
}

const pb::DeviceInfo& StereoModuleHubClientPrivate::device(int index) const
{
    return m_cachedDeviceInfos.info(index);
}

bool StereoModuleHubClientPrivate::deviceAvailable(int index) const
{
    if (!utils::Contains(m_validDeviceIndices, index)) {
        LOG(WARNING) << "No device at dock  " << index;
        return false;
    }

    return true;
}

///------- StereoModuleHubClient starts from here
StereoModuleHubClient::StereoModuleHubClient(const Configs& configs)
    : QObject(), d_ptr(new StereoModuleHubClientPrivate(this))
{
    Q_D(StereoModuleHubClient);
    d->init(configs);
}

StereoModuleHubClient::~StereoModuleHubClient() = default;

StereoModuleHubClient* StereoModuleHubClient::createDefault()
{
    const Configs defaultConfigs{.hostAddress = "localhost:50051",
                                 .maxSendMessageSize = 10,
                                 .maxReceivedMessageSize = 10};

    return new StereoModuleHubClient(defaultConfigs);
}

bool StereoModuleHubClient::isChannelReady()
{
    Q_D(StereoModuleHubClient);
    const auto state = d->m_channel->GetState(false);
    LOG(INFO) << "Current channel state: " << state;
    return state == GRPC_CHANNEL_READY;
}

bool StereoModuleHubClient::tryToConnect(int64_t timeout)
{
    Q_D(StereoModuleHubClient);
    if (isChannelReady()) {
        return true;
    }

    const auto deadline =
        gpr_time_add(gpr_now(GPR_CLOCK_REALTIME),
                     gpr_time_from_seconds(timeout, GPR_TIMESPAN));
    if (!d->m_channel->WaitForConnected(deadline)) {
        LOG(WARNING) << "Failed to connect channel.";
        return false;
    }

    d->m_stub = std::move(pb::CameraController::NewStub(d->m_channel));
    return true;
}

int StereoModuleHubClient::onlineDeviceCount(bool update)
{
    Q_D(StereoModuleHubClient);
    if (update) {
        d->updateCurrentDevices();
    }

    return d->m_validDeviceIndices.size();
}

std::vector<StereoModuleInfo> StereoModuleHubClient::onlineDeviceInfos(
    bool update)
{
    Q_D(StereoModuleHubClient);
    if (update) {
        d->updateCurrentDevices();
    }

    std::vector<StereoModuleInfo> infos;
    for (int i{0}; i < d->m_cachedDeviceInfos.info_size(); ++i) {
        const auto& pbInfo = d->m_cachedDeviceInfos.info(i);
        if (pbInfo.status()) {
            infos.push_back(toStereoModuleInfo(pbInfo));
        }
    }

    return infos;
}

std::vector<StereoModuleInfo> StereoModuleHubClient::deviceInfos(bool update)
{
    Q_D(StereoModuleHubClient);
    if (update) {
        d->updateCurrentDevices();
    }

    std::vector<StereoModuleInfo> infos;
    for (int i{0}; i < d->m_cachedDeviceInfos.info_size(); ++i) {
        infos.push_back(toStereoModuleInfo(d->m_cachedDeviceInfos.info(i)));
    }

    return infos;
}

const std::vector<int>& StereoModuleHubClient::onlineDeviceIndices(bool update)
{
    Q_D(StereoModuleHubClient);
    if (update) {
        d->updateCurrentDevices();
    }

    return d->m_validDeviceIndices;
}

StereoModuleInfo StereoModuleHubClient::deviceInfo(int index, bool update)
{
    Q_D(StereoModuleHubClient);
    if (update) {
        d->updateCurrentDevices();
    }

    if (!d->deviceAvailable(index)) {
        return {};
    }

    return toStereoModuleInfo(d->device(index));
}

int StereoModuleHubClient::devicePosOf(int index, bool update)
{
    Q_D(StereoModuleHubClient);
    if (update) {
        d->updateCurrentDevices();
    }

    return utils::FindPos(d->m_validDeviceIndices, index);
}

int StereoModuleHubClient::deviceIndexAt(int pos, bool update)
{
    Q_D(StereoModuleHubClient);
    if (update) {
        d->updateCurrentDevices();
    }

    if (pos < d->m_validDeviceIndices.size() && pos >= 0) {
        return d->m_validDeviceIndices[pos];
    }

    return -1;
}

StereoImageData StereoModuleHubClient::getStereoImage(int index)
{
    Q_D(StereoModuleHubClient);
    if (!d->deviceAvailable(index)) {
        return {};
    }

    pb::StereoImageData response;
    ::grpc::ClientContext context;
    const auto status =
        d->m_stub->GetImage(&context, d->device(index), &response);
    if (status.ok()) {
        LOG(INFO) << "Received stereo data from dock " << index << ", at "
                  << response.left_timestamp();
        return {bytesToCvMat(response.left_image_data()).clone(),
                bytesToCvMat(response.right_image_data()).clone(),
                response.left_timestamp()};
    }

    LOG(WARNING) << "Failed to received stereo data from dock " << index << ": "
                 << status.error_message();
    return {};
}

void StereoModuleHubClient::bindStereoImageDataStream(
    StereoImageDataStream* stream)
{
    Q_D(StereoModuleHubClient);
    if (!stream) {
        return;
    }

    d->m_stereoImageDataStream = stream;
}

void StereoModuleHubClient::startStreamImages(int index)
{
    Q_D(StereoModuleHubClient);
    if (!d->deviceAvailable(index)) {
        return;
    }

    ::grpc::ClientContext context;
    // const auto deadline = gpr_time_add(gpr_now(GPR_CLOCK_REALTIME),
    //                                    gpr_time_from_seconds(10,
    //                                    GPR_TIMESPAN));
    // context.set_deadline(deadline);
    auto reader = d->m_stub->StartYUVStream(&context, d->device(index));

    LOG(INFO) << "Start stereo data streaming at dock " << index;
    pb::StereoYUVData response;
    while (reader->Read(&response)) {
        // LOG(INFO) << "Received stereo data from dock " << index << ", at "
        //           << response.left_timestamp();
        auto& stream = d->m_stereoImageDataStream->at(index);
        if (stream.closed()) {
            continue;
        }

        const StereoImageData data{bytesToGray(response.left_y()),
                                   bytesToGray(response.right_y()),
                                   response.left_timestamp()};
        // Method: Buffer
        stream << data;
        emit stereoImageDataBufferUpdated(index);

        // Method: Direct emit
        // emit stereoImageDataReceived(data, index);
    }

    const auto status = reader->Finish();
    if (status.ok()) {
        LOG(INFO) << "Streaming stereo data finished at dock " << index;
        emit streamStereoFinished(index, false);
    }
    else {
        LOG(WARNING) << "Streaming stereo data error at dock " << index << ": "
                     << status.error_message();
        emit streamStereoFinished(index, true);
    }
}

void StereoModuleHubClient::stopStreamImages(int index)
{
    Q_D(StereoModuleHubClient);
    if (!d->deviceAvailable(index)) {
        return;
    }

    pb::CaptureResult response;
    ::grpc::ClientContext context;
    const auto status =
        d->m_stub->StopYUVStream(&context, d->device(index), &response);
    if (status.ok()) {
        LOG(INFO) << "Streaming stereo data at dock " << index
                  << " is about to stop.";
    }
    else {
        LOG(WARNING) << "Failed to stop stereo data streaming at dock " << index
                     << ": " << status.error_message();
    }
}

ImuData StereoModuleHubClient::getImuData(int index)
{
    Q_D(StereoModuleHubClient);
    if (!d->deviceAvailable(index)) {
        return {};
    }

    pb::ImuData response;
    ::grpc::ClientContext context;
    const auto status =
        d->m_stub->GetImuData(&context, d->device(index), &response);
    if (status.ok()) {
        LOG(INFO) << "Received IMU data from dock " << index << ", at "
                  << response.timestamp();
        return pbImuDataToImuData(response);
    }

    LOG(WARNING) << "Failed to received IMU data from dock " << index << ": "
                 << status.error_message();
    return {};
}

void StereoModuleHubClient::startStreamImu(int index)
{
    Q_D(StereoModuleHubClient);
    if (!d->deviceAvailable(index)) {
        return;
    }

    pb::Empty request;
    ::grpc::ClientContext context;
    const auto reader =
        d->m_stub->StartImuDataStream(&context, d->device(index));

    LOG(INFO) << "Start IMU data streaming at dock " << index;
    pb::ImuDataList response;
    while (reader->Read(&response)) {
        // LOG(INFO) << "Received IMU data from dock " << index << ", size "
        //           << response.data_size();
        emit imuDataListReceived(pbImuDataListToImuDataList(response), index);
    }

    const auto status = reader->Finish();
    if (status.ok()) {
        LOG(INFO) << "Streaming IMU data finished at dock " << index;
        emit streamImuFinished(index, false);
    }
    else {
        LOG(WARNING) << "Streaming IMU data error at dock " << index << ": "
                     << status.error_message();
        emit streamImuFinished(index, true);
    }
}

void StereoModuleHubClient::stopStreamImu(int index)
{
    Q_D(StereoModuleHubClient);
    if (!d->deviceAvailable(index)) {
        return;
    }

    pb::CaptureResult response;
    ::grpc::ClientContext context;
    const auto status =
        d->m_stub->StopImuDataStream(&context, d->device(index), &response);
    if (status.ok()) {
        LOG(INFO) << "Streaming IMU data at dock " << index
                  << " is about to stop";
    }
    else {
        LOG(WARNING) << "Failed to stop IMU data streaming at dock " << index
                     << ": " << status.error_message();
    }
}

void StereoModuleHubClient::startReceiveImuStream(int index)
{
    Q_D(StereoModuleHubClient);
    if (!d->deviceAvailable(index)) {
        return;
    }

    pb::Empty request;
    ::grpc::ClientContext context;
    const auto reader = d->m_stub->StartRecvImuData(&context, d->device(index));

    LOG(INFO) << "Start receiving IMU data stream at dock " << index;
    pb::ImuData response;
    while (reader->Read(&response)) {
        // LOG(INFO) << "Received IMU data from dock " << index << ", size "
        //           << response.data_size();
        emit imuDataReceived(pbImuDataToImuData(response), index);
    }

    const auto status = reader->Finish();
    if (status.ok()) {
        LOG(INFO) << "Streaming IMU data finished at dock " << index;
        emit streamImuFinished(index, false);
    }
    else {
        LOG(WARNING) << "Streaming IMU data error at dock " << index << ": "
                     << status.error_message();
        emit streamImuFinished(index, true);
    }
}

void StereoModuleHubClient::stopReceiveImuStream(int index)
{
    Q_D(StereoModuleHubClient);
    if (!d->deviceAvailable(index)) {
        return;
    }

    ::grpc::ClientContext context;
    pb::CaptureResult response;
    const auto status =
        d->m_stub->StopRecvImuData(&context, d->device(index), &response);
    if (status.ok()) {
        LOG(INFO) << "Streaming IMU data at dock " << index
                  << " is about to stop";
    }
    else {
        LOG(WARNING) << "Failed to stop IMU data streaming at dock " << index
                     << ": " << status.error_message();
    }
}

bool StereoModuleHubClient::sendCameraCalibrationToDevice(
    const std::string& result, int index)
{
    Q_D(StereoModuleHubClient);
    if (!d->deviceAvailable(index)) {
        return false;
    }

    pb::CalibrationResult request;
    request.set_target(pb::TARGET_DEVICE);
    request.set_name(d->device(index).name());
    request.set_file_data(result);

    pb::CalibrationResult response;
    ::grpc::ClientContext context;
    const auto status =
        d->m_stub->PushCalibrationResult(&context, request, &response);
    if (status.ok()) {
        LOG(INFO) << "Sent calibration result to device " << index;
        return true;
    }

    LOG(WARNING) << "Failed to send calibration result to device " << index
                 << ": " << status.error_message();
    return false;
}

bool StereoModuleHubClient::pullCameraCalibrationFromDevice(std::string& result,
                                                            int index)
{
    Q_D(StereoModuleHubClient);
    if (!d->deviceAvailable(index)) {
        return false;
    }

    pb::CalibrationResult request;
    request.set_target(pb::TARGET_DEVICE);
    request.set_name(d->device(index).name());

    pb::CalibrationResult response;
    ::grpc::ClientContext context;
    const auto status =
        d->m_stub->PullCalibrationResult(&context, request, &response);
    if (status.ok()) {
        if (response.file_data().empty()) {
            LOG(WARNING) << "Received calibration result from device " << index
                         << " is empty.";
            return false;
        }

        LOG(INFO) << "Received calibration result from device " << index;
        result = response.file_data();
        return true;
    }

    LOG(WARNING) << "Failed to pull calibration result from device " << index
                 << ": " << status.error_message();
    return false;
}

bool StereoModuleHubClient::sendUndistortRectifyMapToDevice(
    const std::string& map1, const std::string& map2, int index)
{
    Q_D(StereoModuleHubClient);
    if (!d->deviceAvailable(index)) {
        return false;
    }

    // NOTE: map1 and map2 are relatively big string, so we need to set
    // larger max send/receive message size of grpc chanel.
    pb::UndistortRectifyMap request;
    request.set_name(d->device(index).name());
    request.set_rectify_map_first(map1);
    request.set_rectify_map_second(map2);

    pb::UndistortRectifyMapResult response;
    ::grpc::ClientContext context;
    const auto status =
        d->m_stub->PushUndistortRectifyMap(&context, request, &response);
    if (status.ok()) {
        LOG(INFO) << "Sent undistort rectify map to device " << index;
        return true;
    }

    LOG(WARNING) << "Failed to send undistort rectify map to device " << index
                 << ": " << status.error_message();
    return false;
}

bool StereoModuleHubClient::requestGenerateUndistortRectifyMapBin(int index)
{
    Q_D(StereoModuleHubClient);
    if (!d->deviceAvailable(index)) {
        return false;
    }

    pb::GenerateResult response;
    ::grpc::ClientContext context;
    const auto status =
        d->m_stub->GenerateGdc(&context, d->device(index), &response);
    if (status.ok()) {
        LOG(INFO) << "Sent request to device " << index
                  << " to generate undistort rectify map bin.";
        if (response.result()) {
            LOG(INFO) << "Undistort rectify map bin generated on device "
                      << index;
            return true;
        }

        LOG(WARNING)
            << "Failed to generate undistort rectify map bin on device "
            << index << ": " << response.msg();
        return false;
    }

    LOG(WARNING) << "Failed to send request to device " << index
                 << "to generate undistort rectify map bin."
                 << status.error_message();
    return false;
}

std::unique_ptr<pb::CameraController::Stub>& StereoModuleHubClient::stub()
{
    Q_D(StereoModuleHubClient);
    return d->m_stub;
}

void StereoModuleHubClient::blockStartStreaming()
{
    Q_D(StereoModuleHubClient);
    for (const auto& index : d->m_validDeviceIndices) {
        const auto imuStreamFuture =
            QtConcurrent::run([this, index]() { startStreamImu(index); });
    }

    for (const auto& index : d->m_validDeviceIndices) {
        const auto imgStreamFuture =
            QtConcurrent::run([this, index]() { startStreamImages(index); });
    }
}

void StereoModuleHubClient::blockStopStreaming()
{
    Q_D(StereoModuleHubClient);
    for (const auto& index : d->m_validDeviceIndices) {
        stopStreamImages(index);
        stopStreamImu(index);
    }
}

void StereoModuleHubClient::asyncStartStreaming()
{
    Q_D(StereoModuleHubClient);
    // Create request
    for (const auto& index : d->m_validDeviceIndices) {
        new StreamImuDataRequest(*this, d->device(index), index);
        new StreamStereoImageRequest(*this, d->device(index), index);
    }

    while (pending_requests_) {
        const auto deadline =
            gpr_time_add(gpr_now(GPR_CLOCK_REALTIME),
                         gpr_time_from_seconds(2, GPR_TIMESPAN));

        // Get any IO operation that is ready.
        void* tag{nullptr};
        bool ok = true;

        // Wait for the next event to complete in the queue
        const auto status = m_streamQueue.AsyncNext(&tag, &ok, deadline);
        switch (status) {
            case ::grpc::CompletionQueue::NextStatus::TIMEOUT:
                LOG(ERROR) << "Request next event timed out.";
                continue;
            case ::grpc::CompletionQueue::NextStatus::GOT_EVENT:
                // LOG(INFO) << "Event returned with status " << ok;
                {
                    auto handle = static_cast<AsyncRequest::Handle*>(tag);
                    handle->proceed(ok);
                }
                break;
            case ::grpc::CompletionQueue::NextStatus::SHUTDOWN:
                LOG(INFO) << "Received shutdown event. "
                             "About to tear down gRPC connection. "
                             "Wait for "
                          << handles_in_flight_ << " handles to finish.";
                return;
        }
    }

    LOG(INFO) << "Exiting event-loop";
    assert(handles_in_flight_ == 0);
    closeAsyncStream();
}

void StereoModuleHubClient::asyncStopStreaming()
{
    Q_D(StereoModuleHubClient);
    // TODO
}

void StereoModuleHubClient::closeAsyncStream()
{
    // Make sure we don't close CompletionQueue more than once
    std::call_once(shutdown_, [this] { m_streamQueue.Shutdown(); });
}

void StereoModuleHubClient::incCounter() { ++pending_requests_; }

void StereoModuleHubClient::decCounter()
{
    assert(pending_requests_ >= 1);
    --pending_requests_;
}

void StereoModuleHubClient::callbackStartStreaming()
{
    const auto future =
        QtConcurrent::run([this]() { callbackStartStreamingImpl(); });
}

void StereoModuleHubClient::callbackStartStreamingImpl()
{
    Q_D(StereoModuleHubClient);
    for (const auto& index : d->m_validDeviceIndices) {
        auto handleImuData = [this, index](ImuDataOrStatus val) {
            if (std::holds_alternative<const pb::ImuDataList*>(val)) {
                auto data = std::get<const pb::ImuDataList*>(val);
                emit imuDataListReceived(pbImuDataListToImuDataList(*data),
                                         index);
            }
            else if (std::holds_alternative<::grpc::Status>(val)) {
                auto status = std::get<::grpc::Status>(val);
                if (status.ok()) {
                    LOG(INFO) << "Streaming IMU data at dock " << index
                              << " finished.";
                }
                else {
                    LOG(INFO) << "Streaming IMU data at dock " << index
                              << " failed: " << status.error_message();
                }
            }
            else {
                assert(false && "Unexpected value type");
            }
        };
        new StartImuDataStreamCallbackRequest(*this, d->device(index),
                                              handleImuData);

        auto handleStereoYUVData = [this, index](YUVDataOrStatus val) {
            if (std::holds_alternative<const pb::StereoYUVData*>(val)) {
                auto data = std::get<const pb::StereoYUVData*>(val);
                emit stereoImageDataReceived(
                    {bytesToGray(data->left_y()), bytesToGray(data->right_y()),
                     data->left_timestamp()},
                    index);
            }
            else if (std::holds_alternative<::grpc::Status>(val)) {
                auto status = std::get<::grpc::Status>(val);
                if (status.ok()) {
                    LOG(INFO) << "Streaming stereo data at dock " << index
                              << " finished.";
                }
                else {
                    LOG(INFO) << "Streaming stereo data at dock " << index
                              << " failed: " << status.error_message();
                }
            }
            else {
                assert(false && "Unexpected value type");
            }
        };
        // TODO: Use std::move or not???
        new StartYUVStreamCallbackRequest(*this, d->device(index),
                                          handleStereoYUVData);
    }

    LOG(INFO) << "Waiting for all streaming requests to finish...";
    m_doneStreaming.get_future().get();
    LOG(INFO) << "All streaming finished.";
}

void StereoModuleHubClient::callbackStopStreaming()
{
    Q_D(StereoModuleHubClient);
    for (const auto& index : d->m_validDeviceIndices) {
        auto afterStopImuDataStream = [index](
                                          const ::grpc::Status& status,
                                          const pb::CaptureResult& response) {
            if (status.ok()) {
                LOG(INFO) << "Streaming IMU data at dock " << index
                          << " is about to stop";
            }
            else {
                LOG(WARNING) << "Failed to stop IMU data streaming at dock "
                             << index << ": " << status.error_message();
            }
        };
        new StopImuDataStreamCallbackRequest(*this, d->device(index),
                                             afterStopImuDataStream);

        auto afterStopYUVStream = [index](const ::grpc::Status& status,
                                          const pb::CaptureResult& response) {
            if (status.ok()) {
                LOG(INFO) << "Streaming stereo data at dock " << index
                          << " is about to stop.";
            }
            else {
                LOG(WARNING) << "Failed to stop stereo data streaming at dock "
                             << index << ": " << status.error_message();
            }
        };
        new StopYUVStreamCallbackRequest(*this, d->device(index),
                                         afterStopYUVStream);
    }
}

} // namespace tl

#include "StereoModuleHubClient.moc"
#include "moc_StereoModuleHubClient.cpp"
