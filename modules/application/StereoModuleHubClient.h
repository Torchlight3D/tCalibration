#pragma once

#include <future>
#include <mutex>

#include <QObject>

#include <tMotion/ImuData>
#include "StereoModuleData.h"

#include <grpcpp/completion_queue.h>
#include "device_router.grpc.pb.h"

namespace tl {

class StereoModuleHubClientPrivate;
class StereoModuleHubClient : public QObject
{
    Q_OBJECT

public:
    struct Configs
    {
        std::string hostAddress{"localhost:50051"};
        int maxSendMessageSize = 10;
        int maxReceivedMessageSize = 10;
    };

    // NOTE: We only use QObject to emit signal, dont need parent
    explicit StereoModuleHubClient(const Configs& configs);
    ~StereoModuleHubClient();

    static StereoModuleHubClient* createDefault();
    inline static constexpr int kMaxSupportDevice{4};

    bool isChannelReady();
    bool tryToConnect(int64_t second = 1);

    int onlineDeviceCount(bool update = true);
    std::vector<StereoModuleInfo> onlineDeviceInfos(bool update = false);
    std::vector<StereoModuleInfo> deviceInfos(bool update = false);
    const std::vector<int>& onlineDeviceIndices(bool update = false);
    StereoModuleInfo deviceInfo(int index, bool update = true);
    int devicePosOf(int index, bool update = false);
    int deviceIndexAt(int pos, bool update = false);

    // NOTE: return gray image from YUV's Y channel. Seems a bit weird???
    StereoImageData getStereoImage(int index);
    void bindStereoImageDataStream(StereoImageDataStream* channel);
    void startStreamImages(int index);
    void stopStreamImages(int index);

    ImuData getImuData(int index);
    void startStreamImu(int index);
    void stopStreamImu(int index);
    void startReceiveImuStream(int index);
    void stopReceiveImuStream(int index);

    bool sendCameraCalibrationToDevice(const std::string& result, int index);
    bool pullCameraCalibrationFromDevice(std::string& result, int index);
    bool sendUndistortRectifyMapToDevice(const std::string& map1,
                                         const std::string& map2, int index);
    bool requestGenerateUndistortRectifyMapBin(int index);

signals:
    void stereoImageDataBufferUpdated(int index);
    void stereoImageDataReceived(const StereoImageData& stereo, int index);
    void imuDataReceived(const ImuData& imu, int index);
    void imuDataListReceived(const std::vector<ImuData>& imu, int index);
    void streamStereoFinished(int index, bool withError);
    void streamImuFinished(int index, bool withError);

public: // TEST
    std::unique_ptr<pb::CameraController::Stub>& stub();

public: // TEST: Block
    void blockStartStreaming();
    void blockStopStreaming();

public: // TEST: Async
    void asyncStartStreaming();
    void asyncStopStreaming();
    void closeAsyncStream();

    void incCounter();
    void decCounter();

    ::grpc::CompletionQueue m_streamQueue;
    std::once_flag shutdown_;
    size_t next_client_id_{0};
    size_t request_count_{0};
    size_t pending_requests_{0};
    size_t handles_in_flight_{0};

public: // TEST: Callback
    void callbackStartStreaming();
    void callbackStartStreamingImpl();
    void callbackStopStreaming();

    std::atomic_size_t m_requestCount; // Not used
    std::atomic_size_t m_runningCount;
    std::promise<void> m_doneStreaming;

private:
    Q_DISABLE_COPY(StereoModuleHubClient)
    Q_DECLARE_PRIVATE(StereoModuleHubClient)
    const QScopedPointer<StereoModuleHubClientPrivate> d_ptr;

    // Async request classes
    friend class StreamStereoImageRequest;
    friend class StopStreamStereoImageRequest;
};

} // namespace tl
