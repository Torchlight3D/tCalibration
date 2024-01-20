#pragma once

#include <string>

#include <opencv2/core/mat.hpp>

#include <tMotion/ImuData>

namespace tl {

// TODO:
// 1. Keep consistent with StereoDataRecorder
// 2. Use already existed data structures
// 3. Now assume no one changes (delete/add/modify) the contents during
// operation, what if it happens
class StereoDataPlayer
{
public:
    explicit StereoDataPlayer(const std::string &rootPath);

    bool isValid() const;

    void setStep(int step);

    void toNext();
    void toPrev();
    void toBegin();
    void toEnd();
    void toIndex(size_t index);
    size_t imageCount() const;

    cv::Mat currentLeft() const;
    cv::Mat currentRight() const;
    double currentTimestamp() const;

    const ImuDatas &imuDatas() const;
    size_t imuCount() const;

private:
    bool validatePath(const std::string &path);
    void loadImuData();

private:
    std::string root_;
    std::vector<std::string> filenames_;
    ImuDatas imu_datas_;
    size_t current_;
    int step_;
};

} // namespace tl
