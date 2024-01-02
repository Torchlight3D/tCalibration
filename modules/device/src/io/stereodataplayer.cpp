#include "stereodataplayer.h"

#include <filesystem>

#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>

#include <csv-parser/csv.hpp>
#include <AxCore/TimeUtils>

namespace thoht {

namespace fs = std::filesystem;

namespace {
constexpr char kLeftFolder[]{"left"};
constexpr char kRightFolder[]{"right"};
constexpr char kMotionFilename[]{"motion.csv"};
constexpr char kPNGSuffix[]{".png"};
} // namespace

namespace key {
constexpr char kSequence[]{"seq"};
constexpr char kTimestamp[]{"timestamp"};
constexpr char kAccX[]{"acc_x"};
constexpr char kAccY[]{"acc_y"};
constexpr char kAccZ[]{"acc_z"};
constexpr char kGyroX[]{"gyro_x"};
constexpr char kGyroY[]{"gyro_y"};
constexpr char kGyroZ[]{"gyro_z"};
} // namespace key

StereoDataPlayer::StereoDataPlayer(const std::string &path) : step_(1)
{
    if (validatePath(path)) {
        loadImuData();
        LOG(INFO) << "Load IMU data size: " << imu_datas_.size()
                  << "\n"
                     "Load image data size: "
                  << filenames_.size();
    }
    else {
        LOG(ERROR) << "Invalid dataset path.";
    }
}

bool StereoDataPlayer::isValid() const
{
    return !filenames_.empty() && !root_.empty();
}

void StereoDataPlayer::setStep(int step)
{
    if (step > 0) {
        step_ = step;
    }
}

void StereoDataPlayer::toNext()
{
    current_ = std::min(filenames_.size() - 1, current_ + step_);
}

void StereoDataPlayer::toPrev()
{
    current_ = std::max(size_t{0}, current_ - step_);
}

void StereoDataPlayer::toBegin() { current_ = 0; }

void StereoDataPlayer::toEnd() { current_ = filenames_.size() - 1; }

void StereoDataPlayer::toIndex(size_t index)
{
    current_ = std::clamp(index, size_t{0}, filenames_.size() - 1);
}

size_t StereoDataPlayer::imageCount() const { return filenames_.size(); }

cv::Mat StereoDataPlayer::currentLeft() const
{
    const auto leftPath =
        fs::path{root_} / kLeftFolder / (filenames_[current_] + kPNGSuffix);
    const auto image = cv::imread(leftPath.string());
    return image;
}

cv::Mat StereoDataPlayer::currentRight() const
{
    const auto rightPath =
        fs::path{root_} / kRightFolder / (filenames_[current_] + kPNGSuffix);
    const auto image = cv::imread(rightPath.string());
    return image;
}

double StereoDataPlayer::currentTimestamp() const
{
    const auto &filename = filenames_[current_];
    return time::nsToS(std::stol(filename));
}

const ImuDatas &StereoDataPlayer::imuDatas() const { return imu_datas_; }

bool StereoDataPlayer::validatePath(const std::string &path)
{
    const fs::path dataDir{path};
    if (!fs::exists(dataDir)) {
        return false;
    }

    const auto motionFile = dataDir / kMotionFilename;
    const auto leftDir = dataDir / kLeftFolder;
    const auto rightDir = dataDir / kRightFolder;
    if (!fs::exists(motionFile) || !fs::exists(leftDir) ||
        !fs::exists(rightDir)) {
        return false;
    }

    std::vector<std::string> leftFileNames;
    for (const auto &entry : fs::directory_iterator{leftDir}) {
        if (!entry.is_regular_file() ||
            entry.path().extension() != kPNGSuffix) {
            continue;
        }

        leftFileNames.push_back(entry.path().stem().string());
    }

    std::vector<std::string> rightFileNames;
    for (const auto &entry : fs::directory_iterator{rightDir}) {
        if (!entry.is_regular_file() ||
            entry.path().extension() != kPNGSuffix) {
            continue;
        }

        rightFileNames.push_back(entry.path().stem().string());
    }

    if (leftFileNames.size() != rightFileNames.size()) {
        return false;
    }

    root_ = path;
    filenames_ = leftFileNames;
    current_ = 0;

    for (const auto &filename :
         std::vector(filenames_.begin(), filenames_.begin() + 15)) {
        LOG(INFO) << "Filename: " << filename;
    }

    return true;
}

void StereoDataPlayer::loadImuData()
{
    const auto motionFile = fs::path{root_} / kMotionFilename;

    csv::CSVReader reader{motionFile.string()};
    for (auto &row : reader) {
        auto timestamp = time::nsToS(row[key::kTimestamp].get<long>());
        imu_datas_.acc.d().emplace_back(
            timestamp, row[key::kAccX].get<double>(),
            row[key::kAccY].get<double>(), row[key::kAccZ].get<double>());
        imu_datas_.gyro.d().emplace_back(
            timestamp, row[key::kGyroX].get<double>(),
            row[key::kGyroY].get<double>(), row[key::kGyroZ].get<double>());
        imu_datas_.timeline.emplace_back(timestamp);
    }
}

} // namespace thoht
