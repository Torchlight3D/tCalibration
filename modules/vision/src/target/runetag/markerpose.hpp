#pragma once

#include <opencv2/opencv.hpp>
#include "markerdetected.hpp"

namespace cv {
namespace runetag {
const unsigned int FLAG_REPROJ_ERROR = 1;
const unsigned int FLAG_REFINE = 2;

/// <summary> Camera pose as a transformation from camera coordinates to world
/// coordinates </summary>
struct Pose
{
    /// <value> Rotation matrix </value>
    cv::Mat R;
    /// <value> Translation vector </value>
    cv::Mat t;
};

extern Pose findPose(const MarkerDetected& detected, const cv::Mat& intrinsics,
                     const cv::Mat& distortion, bool* pose_ok = 0,
                     unsigned int method = CV_ITERATIVE, unsigned int flag = 0);

} // namespace runetag
} // namespace cv
