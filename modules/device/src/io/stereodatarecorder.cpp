#include "stereodatarecorder.h"

#include <filesystem>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <csv-parser/csv.hpp>
#include <nlohmann/json.hpp>

#include <tMotion/ImuData>

#include "framebuffer.h"
#include "future.h"

namespace tl {

namespace fs = std::filesystem;

namespace {
constexpr int kOutputPrecision{10};
constexpr int kImageFileNameWidth{10};
constexpr char kAviExt[]{".avi"};
constexpr char kCsvExt[]{".csv"};
constexpr char kMotionCsvFilename[]{"motion.csv"};
constexpr char kSequence[]{"seq"};
constexpr char kFlag[]{"flag"};
constexpr char kSystemTimestamp[]{"sys_timestamp"};
constexpr char kTimestamp[]{"timestamp"};
constexpr char kTemperature[]{"temperature"};
constexpr char kAccX[]{"acc_x"};
constexpr char kAccY[]{"acc_y"};
constexpr char kAccZ[]{"acc_z"};
constexpr char kGyroX[]{"gyro_x"};
constexpr char kGyroY[]{"gyro_y"};
constexpr char kGyroZ[]{"gyro_z"};
constexpr char kCameraId[]{"camId"};
constexpr char kCameraPrefix[]{"cam"};
constexpr int kImuFieldCount{9};
} // namespace

///------ VideoWriter starts from here
// Similar to CameraMetaData
struct FrameMetaData
{
    long long systemTimestamp{0ll};
    double timestamp{0.};
};

class VideoWriter
{
public:
    using Ptr = std::unique_ptr<VideoWriter>;
    using ConstPtr = std::unique_ptr<const VideoWriter>;

    static Ptr build(const std::string &name, double fps,
                     const cv::Mat &sample);

    void write(const cv::Mat &frame, const FrameMetaData &meta);

    void closOutputfile();

private:
    explicit VideoWriter(std::unique_ptr<cv::VideoWriter> writer,
                         const std::string &path);

private:
    const std::unique_ptr<cv::VideoWriter> m_writer;
    std::ofstream m_textFileOutput;
    std::ostream &m_textOutput;
    cv::Mat m_frameCache;
};

VideoWriter::VideoWriter(std::unique_ptr<cv::VideoWriter> writer,
                         const std::string &path)
    : m_writer(std::move(writer)),
      m_textFileOutput(path),
      m_textOutput(m_textFileOutput)
{
    m_textOutput.precision(kOutputPrecision);
    {
        auto writer = csv::make_csv_writer(m_textOutput);
        const std::vector<std::string> title{kSystemTimestamp, kTimestamp};
        writer << title;
    }
}

std::unique_ptr<VideoWriter> VideoWriter::build(const std::string &name,
                                                double fps,
                                                const cv::Mat &sample)
{
    auto toVideoOutputPath = [](const std::string &name) -> std::string {
        // Must be .avi so OpenCV can record this without FFMPEG
        const auto path = name + kAviExt;
        return path;
    };

    auto toTextOutputPath = [](const std::string &name) -> std::string {
        const auto path = name + kCsvExt;
        return path;
    };

    auto buildCVVideoWriter =
        [](const std::string &path, double fps,
           const cv::Mat &sample) -> std::unique_ptr<cv::VideoWriter> {
        assert(fps > 0.);
        assert(!path.empty());

        // This is the only thing we can write without FFMPEG
        // The path name should end with .avi
        constexpr auto backend = cv::CAP_OPENCV_MJPEG;
        const auto codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        // Force save color, saved video shows glitch if we save gray???
        const bool isColor = true;
        //        const bool isColor = sample.channels() > 1;

        {
            // OpenCV writer gives no errors even if it is unable to open file
            // and write frames to it, so test writing by ourselves.
            std::ofstream video{path};
            assert(video.is_open() && "Unable to open video file for writing");
        }

        auto writer = std::make_unique<cv::VideoWriter>(
            path, backend, codec, fps, sample.size(), isColor);
        assert(writer && "Failed to create video writer");

        // TODO: Maybe set video quality
        return writer;
    };

    return std::unique_ptr<VideoWriter>(new VideoWriter(
        buildCVVideoWriter(toVideoOutputPath(name), fps, sample),
        toTextOutputPath(name)));
}

void VideoWriter::write(const cv::Mat &frame, const FrameMetaData &meta)
{
    if (frame.channels() == 4) {
        cv::cvtColor(frame, m_frameCache, cv::COLOR_BGRA2BGR);
        // This took a while to debug: if the image has 3 channels, the
        // default channel order assumed by OpenCV image IO functions
        // is BGR (which everybody on the internet warns you about).
        // However, if there are 4 channels, at least this particular
        // function assumes the color order RGBA
        m_writer->write(m_frameCache);
    }
    else {
        cv::cvtColor(frame, m_frameCache, cv::COLOR_GRAY2BGR);
        m_writer->write(m_frameCache);
    }

    auto writer = csv::make_csv_writer(m_textFileOutput);
    writer << std::make_tuple(meta.systemTimestamp, meta.timestamp);
}

void VideoWriter::closOutputfile() { m_textFileOutput.close(); }

///------- StereoDataRecorder::Impl starts from here
using json = nlohmann::json;

namespace key {
constexpr char kTimestamp[]{"time"};
constexpr char kSensor[]{"sensor"};
constexpr char kReading[]{"values"};
constexpr char kTemperature[]{"temperature"};
constexpr char kFrames[]{"frames"};
constexpr char kFrameNumber[]{"frame"};
constexpr char kCameraId[]{"camera_id"};
constexpr char kIntrinsics[]{"intrinsics"};
constexpr char kFocalLengthX[]{"fx"};
constexpr char kFocalLengthY[]{"fy"};
constexpr char kPrincipalPointX[]{"px"};
constexpr char kPrincipalPointY[]{"py"};
} // namespace key

class StereoDataRecorder::Impl
{
public:
    explicit Impl(const std::string &outputPath);

    void init();

    void setFrame(const FrameData &frame);

    bool getEmptyFrames(size_t number, double time, int width, int height,
                        int type, std::vector<cv::Mat> &out);
    bool allocateAndWriteVideo(const std::vector<FrameData> &frames,
                               bool clone);

public:
    std::string m_saveDir;
    std::ofstream m_motionFileOutput;
    std::ostream &m_motionOutput;
    std::unique_ptr<Processor> m_motionProcessor;

    std::map<int, std::unique_ptr<VideoWriter>> m_videoWriters;
    std::map<int, std::unique_ptr<Processor>> m_visualProcessors;
    std::unique_ptr<FrameBuffer> m_frameBuffer;
    std::vector<cv::Mat> allocatedFrames;
    double m_fps = 10.;
};

// FIXME: Now assume outputPath is valid
StereoDataRecorder::Impl::Impl(const std::string &saveDir)
    : m_saveDir(saveDir),
      m_motionFileOutput((fs::path{saveDir} / kMotionCsvFilename).string()),
      m_motionOutput(m_motionFileOutput)
{
}

void StereoDataRecorder::Impl::init()
{
    m_motionOutput.precision(kOutputPrecision);

    {
        auto writer = csv::make_csv_writer(m_motionOutput);
        const std::vector<std::string> title{
            kSystemTimestamp, kTimestamp, kAccX,  kAccY,       kAccZ,
            kGyroX,           kGyroY,     kGyroZ, kTemperature};
        writer << title;
    }

    m_motionProcessor = Processor::createThreadPool(1);

    constexpr size_t kCapacityInc = 4;
    // Shared between stereo, i.e. MAX_CAPACITY mono frames, or MAX_CAPACITY/2
    // stereo pairs can be buffered in memory before frame skipping occurs if
    // video encoding cannot keep up
    constexpr size_t kMaxCapacity = 20;
    m_frameBuffer = std::make_unique<FrameBuffer>(kCapacityInc, kMaxCapacity);
}

void StereoDataRecorder::Impl::setFrame(const FrameData &frame)
{
    //  TODO:
}

bool StereoDataRecorder::Impl::getEmptyFrames(size_t number, double time,
                                              int width, int height, int type,
                                              std::vector<cv::Mat> &out)
{
    out.resize(number);
    for (size_t i{0}; i < number; i++) {
        auto frame = m_frameBuffer->next(height, width, type);
        if (!frame) {
            // frameDrop(time);
            out.clear(); // Free already allocated frames
            return false;
        }
        out[i] = *frame;
    }
    return true;
}

bool StereoDataRecorder::Impl::allocateAndWriteVideo(
    const std::vector<FrameData> &frames, bool deepCopy)
{
    allocatedFrames.clear();
    // Allocate all frames, so if we don't have space for second frame of
    // stereo, drop both
    for (const auto &frame : frames) {
        if (!frame.frameData) {
            continue;
        }

        auto framePtr =
            m_frameBuffer->next(frame.frameData->rows, frame.frameData->cols,
                                frame.frameData->type());
        if (!framePtr) {
            return false;
        }

        cv::Mat allocatedFrameData = *framePtr.get();
        if (deepCopy) {
            frame.frameData->copyTo(allocatedFrameData);
        }
        else {
            // Wont copy data, cv::Mat behaves like smart pointer
            allocatedFrameData = *frame.frameData;
        }
        allocatedFrames.push_back(allocatedFrameData);
    }

    for (size_t i{0}; i < frames.size(); i++) {
        const auto &frame = frames[i];
        if (!frame.frameData) {
            continue;
        }

        cv::Mat allocatedFrameData = allocatedFrames[i];
        const int camId = frame.cameraId;
        if (!m_videoWriters.count(camId)) {
            const auto videoPath =
                fs::path{m_saveDir} / (kCameraPrefix + std::to_string(camId));
            m_videoWriters[camId] = VideoWriter::build(
                videoPath.string(), m_fps, allocatedFrameData);
            m_visualProcessors[camId] = Processor::createThreadPool(1);
        }

        m_visualProcessors.at(camId)->enqueue(
            [this, camId, allocatedFrameData, t = frame.t]() {
                m_videoWriters.at(camId)->write(
                    allocatedFrameData,
                    FrameMetaData{
                        .systemTimestamp = std::chrono::steady_clock::now()
                                               .time_since_epoch()
                                               .count(),
                        .timestamp = t});
            });
    }
    return true;
}

///------- StereoDataRecorder starts from here
StereoDataRecorder::StereoDataRecorder(const std::string &outputPath)
    : d(std::make_unique<Impl>(outputPath))
{
    d->init();
}

StereoDataRecorder::~StereoDataRecorder() = default;

StereoDataRecorder::Ptr StereoDataRecorder::create(const std::string &saveDir)
{
    fs::create_directories(fs::path{saveDir});
    return std::unique_ptr<StereoDataRecorder>(new StereoDataRecorder(saveDir));
}

void StereoDataRecorder::setVideoRecordingFps(double fps) { d->m_fps = fps; }

void StereoDataRecorder::addMotionData(const ImuData &data)
{
    auto saveImpl = [this, &data]() {
        const auto &acc = data.acc;
        const auto &gyro = data.gyro;
        const auto now =
            std::chrono::steady_clock::now().time_since_epoch().count();
        auto writer = csv::make_csv_writer(d->m_motionFileOutput);
        writer << std::array<std::string, kImuFieldCount>{
            std::to_string(now),
            std::to_string(acc.timestamp()),
            std::to_string(acc.x()),
            std::to_string(acc.y()),
            std::to_string(acc.z()),
            std::to_string(gyro.x()),
            std::to_string(gyro.y()),
            std::to_string(gyro.z()),
            std::to_string(data.temperature)};
    };

    // FIXME: If we use multi-thread here, some of the ImuData becomes 0.
    saveImpl();
    // d->m_motionProcessor->enqueue(saveImpl);
}

bool StereoDataRecorder::addFrame(const FrameData &frame, bool deepCopy)
{
    // TODO: Check path
    const std::vector<FrameData> &frames{frame};
    return d->allocateAndWriteVideo(frames, deepCopy);
}

bool StereoDataRecorder::addFrameGroup(double t,
                                       const std::vector<FrameData> &frames,
                                       bool deepCopy)
{
    return d->allocateAndWriteVideo(frames, deepCopy);
}

void StereoDataRecorder::closeOutputFile() { d->m_motionFileOutput.close(); }

} // namespace tl
