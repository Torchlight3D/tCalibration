#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace tl {

class DistortionVerifier
{
public:
    bool measure(const cv::Mat &image, double &distortionError,
                 double &maxDistortionError);

    // TODO: remove these two
    const cv::Mat &lineResult() const;
    const cv::Mat &subsamplesResult() const;

private:
    cv::Mat _result;
    cv::Mat _subsample_result;
};

// TODO: Add helper

} // namespace tl
