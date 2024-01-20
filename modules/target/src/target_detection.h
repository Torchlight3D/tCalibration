#pragma once

#include <vector>
#include <opencv2/core/types.hpp>

#include "target_types.h"

namespace tl {

struct TargetDetection
{
    // Detected corners Ids.
    std::vector<CornerId> cornerIds;

    // Detected corners
    std::vector<cv::Point2d> corners;

    // Most of the time, we skip invalid Detections. But they can be useful in
    // error analysis.
    bool valid = false;

    inline int cornerCount() const { return static_cast<int>(corners.size()); }
};

using TargetDetections = std::vector<TargetDetection>;

struct StampedTargetDetection
{
    // Detection
    TargetDetection detection;

    // Timestamp, in second.
    double t = 0.;

    // TODO: Add move semantic ctor
    StampedTargetDetection(const TargetDetection &detection, double t)
        : detection(detection), t(t)
    {
    }

    // for convenience
    const auto &cornerIds() const { return detection.cornerIds; }
    const auto &corners() const { return detection.corners; }
    const auto &valid() const { return detection.valid; }
    auto cornerCount() const { return detection.cornerCount(); }
};

using StampedTargetDetections = std::vector<StampedTargetDetection>;

} // namespace tl
