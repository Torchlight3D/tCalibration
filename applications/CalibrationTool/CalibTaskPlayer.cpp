#include "CalibTaskPlayer.h"

#include <filesystem>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <QtConcurrent/QtConcurrent>

#include <csv-parser/csv.hpp>

#include <tCalib/IO/CalibrationIO>
#include <tCalib/CalibrationData>
#include <tCore/ContainerUtils>
#include <tCore/FileUtils>
#include <tMotion/ImuData>

#include "DeviceInfo.h"

namespace tl {

namespace fs = std::filesystem;

namespace {
// FIXME: Duplicated
constexpr char kLeftVisualFilename[]{"cam0.avi"};
constexpr char kRightVisualFilename[]{"cam1.avi"};
constexpr char kLeftVisualMetaFilename[]{"cam0.csv"};
constexpr char kRightVisualMetaFilename[]{"cam1.csv"};
constexpr char kMotionFilename[]{"motion.csv"};

inline int getVideoFrameCount(const cv::VideoCapture &video)
{
    return video.isOpened()
               ? static_cast<int>(video.get(cv::CAP_PROP_FRAME_COUNT))
               : -1;
}

fs::path verifyDataDir(const fs::path &root)
{
    // Ver.2
    if (const auto dataDir = root / io::kVerifySamplesDir;
        fs::exists(dataDir)) {
        return dataDir;
    }
    // Ver.1
    return root;
}

// TODO: Not complete. Add final action
class ScopeCounter
{
public:
    explicit ScopeCounter(int &prev, int step = 1) : prev(prev), step(step) {}
    ~ScopeCounter() { prev += step; }

private:
    int &prev;
    int step{1};
};

} // namespace

CalibTaskInfo::CalibTaskInfo(const std::string &path) {}

bool CalibTaskInfo::checkPath(const std::string &path, Type &type)
{
    const fs::path root{path};

    // Cameras data
    return false;
}

bool CalibTaskInfo::validate()
{
    //
    return false;
}
bool CalibTaskInfo::isValid() const { return !_path.empty() && _type != None; }

std::string CalibTaskInfo::camerasDir(const std::string &sub) const
{
    return (fs::path{_path} / io::kCamerasDir / sub).string();
}

std::string CalibTaskInfo::verifySamplesDir(const std::string &sub) const
{
    return (fs::path{_path} / io::kVerifySamplesDir / sub).string();
}

std::string CalibTaskInfo::calibParameterFilename() const
{
    return (fs::path{_path} / io::kCalibCamParametersFilename).string();
}

std::string CalibTaskInfo::playbackDir(const std::string &sub) const
{
    return (fs::path{_path} / io::kPlaybackDir / sub).string();
}

CalibTaskPlayer::CalibTaskPlayer(const std::string &path) { parsePath(path); }

bool CalibTaskPlayer::isValid() const { return m_infos.isValid(); }

const CalibTaskPlayer::Infos &CalibTaskPlayer::infos() const { return m_infos; }

bool CalibTaskPlayer::parsePath(const std::string &path)
{
    const fs::path taskPath{path};
    if (!fs::exists(taskPath) || !fs::is_directory(taskPath)) {
        return false;
    }

    // 1. Check if neccessary data exist
    // TODO: Deplicated code
    const auto camDir = [&taskPath]() {
        // Check if Ver.2?
        if (const auto dir = taskPath / io::kCamerasDir; fs::exists(dir)) {
            return dir;
        }
        return taskPath;
    }();

    const auto calibParams = taskPath / io::kCalibCamParametersFilename;
    const auto leftVisual = camDir / kLeftVisualFilename;
    const auto rightVisual = camDir / kRightVisualFilename;
    const auto leftMeta = camDir / kLeftVisualMetaFilename;
    const auto rightMeta = camDir / kRightVisualMetaFilename;
    const auto motion = camDir / kMotionFilename;
    if (!fs::exists(calibParams) || !fs::exists(leftVisual) ||
        !fs::exists(rightVisual) || !fs::exists(leftMeta) ||
        !fs::exists(rightMeta) || !fs::exists(motion)) {
        return false;
    }

    // TODO:
    // 1. No easy way to check if meta data or motion data is valid.
    // 2. Maybe check visual timestamp later.
    {
        const cv::VideoCapture left{leftVisual.string()};
        const cv::VideoCapture right{rightVisual.string()};
        const auto leftFrameCount = getVideoFrameCount(left);
        qDebug() << "Try to load task with frame count: " << leftFrameCount;
        if (!left.isOpened() || !right.isOpened() || (leftFrameCount == -1) ||
            (leftFrameCount != getVideoFrameCount(right))) {
            return false;
        }
    }

    m_infos.params = calibParams.string();
    m_infos.left = leftVisual.string();
    m_infos.right = rightVisual.string();
    m_infos.leftMeta = leftMeta.string();
    m_infos.rightMeta = rightMeta.string();
    m_infos.motion = motion.string();

    // 2. Check verify data. Not used yet
    const auto verifyDir = [&taskPath]() {
        // Check if Ver.2?
        if (const auto dir = taskPath / io::kVerifySamplesDir;
            fs::exists(dir)) {
            return dir;
        }
        return taskPath;
    }();

    return isValid();
}

///------- GroupCalibTaskPlayer starts from here
GroupCalibTaskPlayer::GroupCalibTaskPlayer(const std::string &path,
                                           QObject *parent)
    : QObject(parent), m_path(path)
{
}

bool GroupCalibTaskPlayer::loadTasks() { return parsePath(m_path); }

bool GroupCalibTaskPlayer::isValid() const { return !m_players.empty(); }

const std::string &GroupCalibTaskPlayer::root() const { return m_path; }

int GroupCalibTaskPlayer::count() const
{
    return static_cast<int>(m_players.size());
}

int GroupCalibTaskPlayer::currentIndex() const { return m_index; }

void GroupCalibTaskPlayer::playCurrent()
{
    if (!isValid()) {
        return;
    }

    dockCurrent();
    [[maybe_unused]] const auto _ =
        QtConcurrent::run(&GroupCalibTaskPlayer::streamCurrent, this);
}

void GroupCalibTaskPlayer::toPrevTask()
{
    if (!isValid()) {
        return;
    }

    m_index = std::max(--m_index, 0);
    playCurrent();
}

void GroupCalibTaskPlayer::toNextTask()
{
    if (!isValid()) {
        return;
    }

    m_index = std::min(++m_index, count() - 1);
    playCurrent();
}

void GroupCalibTaskPlayer::toTask(int index)
{
    if (!isValid()) {
        return;
    }

    m_index = std::clamp(index, 0, count() - 1);
    playCurrent();
}

bool GroupCalibTaskPlayer::finished() const { return m_index == (count() - 1); }

bool GroupCalibTaskPlayer::parsePath(const std::string &path)
{
    const fs::path rootPath{path};
    if (!fs::exists(rootPath) || !fs::is_directory(rootPath)) {
        return false;
    }

    const auto total = file::dirCountUnder(rootPath);
    int count{0};
    for (const auto &entry : fs::directory_iterator{rootPath}) {
        ++count;
        const auto &taskPath = entry.path();
        if (!entry.is_directory() ||
            !taskPath.stem().string().starts_with("task")) {
            emit loadingProgress(count, total);
            continue;
        }

        const CalibTaskPlayer player{taskPath.string()};
        if (player.isValid()) {
            m_players.push_back(player);
        }
        emit loadingProgress(count, total);
    }

    if (!m_players.empty()) {
        m_index = 0;
    }

    return isValid();
}

void GroupCalibTaskPlayer::dockCurrent()
{
    if (!isValid()) {
        return;
    }

    const auto &player = m_players[m_index];
    const auto &infos = player.infos();

    // Meta
    CalibMetaData meta;
    io::loadMetaFromFile(infos.params, meta);

    emit taskLoaded(DeviceInfo{.name = meta.uuid,
                               .hostAddr = "localhost",
                               .hardwareAddr = meta.hardwareAddr,
                               .sn = meta.deviceSN,
                               .isVirtual = true});
}

void GroupCalibTaskPlayer::streamCurrent()
{
    if (!isValid()) {
        return;
    }

    const auto &player = m_players[m_index];
    const auto &infos = player.infos();

    // TODO: Try to keep the keys consistent with Recorder
    // Stream visual
    {
        cv::VideoCapture left{infos.left};
        cv::VideoCapture right{infos.right};
        csv::CSVReader leftMetaReader{infos.leftMeta};

        const auto frameCount = getVideoFrameCount(left);
        for (int i{0}; i < frameCount; ++i) {
            StereoImages stereo;
            left >> stereo.left;
            right >> stereo.right;

            // NOTE: No idea why sometimes
            if (stereo.left.empty() || stereo.right.empty()) {
                // Skip this line as well
                csv::CSVRow row;
                leftMetaReader.read_row(row);
            }
            else {
                cv::cvtColor(stereo.left, stereo.left, cv::COLOR_BGR2GRAY);
                cv::cvtColor(stereo.right, stereo.right, cv::COLOR_BGR2GRAY);
                csv::CSVRow row;
                if (leftMetaReader.read_row(row)) {
                    stereo.timestamp = row["timestamp"].get<double>();
                }
                emit stereoImageDataReceived(stereo);
            }
        }
    }

    // Stream motion
    {
        csv::CSVReader motionReader{infos.motion};
        for (auto &row : motionReader) {
            const auto t = row["timestamp"].get<double>();
            emit imuDataReceived(ImuData{
                .acc = {t, row["acc_x"].get<double>(),
                        row["acc_y"].get<double>(), row["acc_z"].get<double>()},
                .gyro = {t, row["gyro_x"].get<double>(),
                         row["gyro_y"].get<double>(),
                         row["gyro_z"].get<double>()},
                .temperature = 0.});
        }
    }

    emit streamDataFinished();
}

///------- VerifyTaskPlayer starts from here
VerifyTaskPlayer::VerifyTaskPlayer(const std::string &path) { parsePath(path); }

bool VerifyTaskPlayer::isValid() const
{
    return !m_suffixes.empty() && !m_root.empty();
}

size_t VerifyTaskPlayer::count() const { return m_suffixes.size(); }

std::string VerifyTaskPlayer::name() const
{
    return fs::path{m_root}.stem().string();
}

void VerifyTaskPlayer::playCurrent()
{
    // TODO
}

void VerifyTaskPlayer::playPrev()
{
    if (!isValid()) {
        return;
    }

    m_index = std::max(--m_index, size_t{0});
}

void VerifyTaskPlayer::playNext()
{
    if (!isValid()) {
        return;
    }

    // Play normal, stop at the last one
    m_index = std::min(++m_index, count() - 1);
}

void VerifyTaskPlayer::toIndex(size_t index)
{
    if (index >= m_suffixes.size()) {
        return;
    }

    m_index = index;
}

bool VerifyTaskPlayer::finished() const
{
    if (!isValid()) {
        return false;
    }

    return m_index == (count() - 1);
}

VerifyTaskPlayer::Infos VerifyTaskPlayer::currentInfos() const
{
    if (!isValid()) {
        return {};
    }

    const auto &suffix = m_suffixes[m_index];

    const fs::path taskDir{m_root};

    const auto dataDir = verifyDataDir(taskDir);

    const auto reportName = [&]() {
        // Ver.4
        if (const auto file =
                dataDir /
                std::format("{}_{}.tt", kVerifyReportPrefixV2, suffix);
            fs::exists(file)) {
            return file.string();
        }

        // Ver.3
        if (const auto file =
                dataDir /
                std::format("{}_{}.tt", kVerifyReportPrefixV1, suffix);
            fs::exists(file)) {
            return file.string();
        }

        // Ver.1 & 2
        return (dataDir /
                std::format("{}_{}.txt", kVerifyReportPrefixV1, suffix))
            .string();
    }();

    return {
        .parameters = (taskDir / io::kVerifyCamParametersFilename).string(),
        .left = (dataDir / std::format("verify_left_{}.png", suffix)).string(),
        .right =
            (dataDir / std::format("verify_right_{}.png", suffix)).string(),
        .report = reportName};
}

int VerifyTaskPlayer::currentIndex() const { return m_index; }

bool VerifyTaskPlayer::parsePath(const std::string &path)
{
    const fs::path taskDir{path};
    if (!fs::exists(taskDir) ||
        !fs::exists(taskDir / io::kVerifyCamParametersFilename)) {
        return false;
    }

    auto getReportPrefix =
        [](const std::string &filename) -> std::optional<std::string> {
        if (filename.starts_with(kVerifyReportPrefixV2)) {
            return kVerifyReportPrefixV2;
        }
        if (filename.starts_with(kVerifyReportPrefixV1)) {
            return kVerifyReportPrefixV1;
        }
        return {};
    };

    const auto dataDir = verifyDataDir(taskDir);
    for (const auto &entry : fs::directory_iterator{dataDir}) {
        const auto &entryPath = entry.path();
        const auto entryName = entryPath.stem().string();

        // Report filename possible patterns:
        // 1. "verification_report_index.txt"
        // 2. "verification_report_index_yyMMddhhmmss.txt"
        // 3. "verification_report_index_yyMMddhhmmss.tt"
        // 4. "verify_report_yyMMddhhmmss.tt"
        // Corresponding sample image pairs share the same suffix.
        // We assume if any verify report is found, there must exist
        // corresponding sample image pair.

        const auto reportPrefix = getReportPrefix(entryName);
        if (!entry.is_regular_file() || !reportPrefix) {
            continue;
        }

        m_suffixes.push_back(entryName.substr(reportPrefix.value().size() + 1));
    }

    if (!m_suffixes.empty()) {
        m_index = 0;
        m_root = path;
    }

    return isValid();
}

///------- GroupVerifyTaskPlayer starts from here
GroupVerifyTaskPlayer::GroupVerifyTaskPlayer(const std::string &path,
                                             QObject *parent)
    : QObject(parent)
{
    parsePath(path);
}

bool GroupVerifyTaskPlayer::isValid() const { return !m_players.empty(); }

void GroupVerifyTaskPlayer::toPrevTask()
{
    toTask(m_index == 0 ? 0 : m_index - 1);
}

void GroupVerifyTaskPlayer::toNextTask()
{
    toTask(std::min(m_index + 1, taskCount() - 1));
}

void GroupVerifyTaskPlayer::toTask(size_t index)
{
    if (index >= taskCount()) {
        return;
    }

    m_index = index;

    emit taskLoaded(m_players[index].name(), m_players[index].count());
}

size_t GroupVerifyTaskPlayer::currentTaskIndex() const { return m_index; }

size_t GroupVerifyTaskPlayer::taskCount() const { return m_players.size(); }

void GroupVerifyTaskPlayer::playCurrent()
{
    if (!isValid()) {
        return;
    }

    // Load current
    loadCurrentFrame();
}

// FIXME: Special care for [Starting/Ending/Transition] indices.
// Make sure no order or redundant problems.
void GroupVerifyTaskPlayer::playPrev()
{
    if (!isValid()) {
        return;
    }

    // Load current
    loadCurrentFrame();

    // Move to previous data
    auto &player = m_players[m_index];
    if (player.finished()) {
        toPrevTask();
    }
    else {
        player.playPrev();
    }
}

void GroupVerifyTaskPlayer::playNext()
{
    if (!isValid()) {
        return;
    }

    // Load current
    loadCurrentFrame();

    // Move to next data
    auto &player = m_players[m_index];
    if (player.finished()) {
        toNextTask();
    }
    else {
        player.playNext();
    }
}

void GroupVerifyTaskPlayer::playIndex(size_t index)
{
    m_players[m_index].toIndex(index);
}

size_t GroupVerifyTaskPlayer::frameCount() const
{
    return m_players[m_index].count();
}

bool GroupVerifyTaskPlayer::finished() const
{
    return !isValid() || (m_forward && m_index == taskCount() - 1 ||
                          !m_forward && m_index == 0);
}

bool GroupVerifyTaskPlayer::parsePath(const std::string &path)
{
    const fs::path savedDataDir{path};
    if (!fs::exists(savedDataDir) || !fs::is_directory(savedDataDir)) {
        return false;
    }

    for (const auto &entry : fs::directory_iterator{savedDataDir}) {
        const auto &taskDir = entry.path();
        if (!entry.is_directory() ||
            !taskDir.stem().string().starts_with("task")) {
            continue;
        }

        if (const VerifyTaskPlayer player{taskDir.string()}; player.isValid()) {
            m_players.push_back(player);
        }
    }

    if (!m_players.empty()) {
        m_index = 0;
        m_forward = true;
    }

    return isValid();
}

void GroupVerifyTaskPlayer::loadCurrentFrame()
{
    const auto &player = m_players[m_index];

    const auto infos = player.currentInfos();

    std::string params;
    {
        std::ifstream fin{infos.parameters};
        if (!fin.is_open()) {
            return;
        }

        std::ostringstream oss;
        oss << fin.rdbuf();
        params = oss.str();
    }

    emit stereoDataReceived(params, {.left = cv::imread(infos.left),
                                     .right = cv::imread(infos.right),
                                     .timestamp = 1.});
}

} // namespace tl

#include "moc_CalibTaskPlayer.cpp"
