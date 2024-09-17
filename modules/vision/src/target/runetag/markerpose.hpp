#pragma once

#include <opencv2/calib3d.hpp>

#include "markerdetected.hpp"

namespace tl {
namespace runetag {

constexpr unsigned int FLAG_REPROJ_ERROR = 1;
constexpr unsigned int FLAG_REFINE = 2;

// Camera pose as a transformation from camera coordinates to world coordinates
struct Pose
{
    cv::Mat R, t;
};

Pose findPose(const MarkerDetected& detected, const cv::Mat& intrinsics,
              const cv::Mat& distortion, bool* pose_ok = 0,
              unsigned int method = cv::SOLVEPNP_ITERATIVE,
              unsigned int flag = 0);

} // namespace runetag
} // namespace tl
