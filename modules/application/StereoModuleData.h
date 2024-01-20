#pragma once

#include <opencv2/core/mat.hpp>
#include <tCore/Channel>

namespace tl {

struct StereoModuleInfo
{
    std::string name{};
    std::string maskedIp{};
    std::string hardwareAddr{};
    std::string sn{};
    bool connected{false};
};

struct StereoImageData
{
    cv::Mat left;
    cv::Mat right;
    double timestamp{0.};

    StereoImageData() {}
    StereoImageData(const cv::Mat& left, const cv::Mat& right, double timestamp)
        : left(left), right(right), timestamp(timestamp)
    {
    }

    StereoImageData deepCopy()
    {
        StereoImageData data;
        left.copyTo(data.left);
        right.copyTo(data.right);
        data.timestamp = timestamp;
        return data;
    }

    bool isValid() const;
};

using StereoImageDataStream = std::array<Channel<StereoImageData>, 4>;

} // namespace tl
