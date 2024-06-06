#pragma once

#include <vector>

#include <opencv2/core/types.hpp>

#include "types.h"

namespace tl {

struct TargetDetection
{
    // Corners
    std::vector<cv::Point2d> corners;

    // Corners Ids
    std::vector<CornerId> cornerIds;

    // If corners are detected
    std::vector<uchar> detected;

    // Because all the possible points, no matter detected or not, are saved.
    // An extra flag is needed to indentify if this detection is valid.
    int count = 0;

    inline bool valid() const { return count > 0; }
    inline bool complete() const
    {
        return valid() && count == static_cast<int>(cornerIds.size());
    }
    inline size_t total() const { return corners.size(); }
};

using TargetDetections = std::vector<TargetDetection>;

struct StampedTargetDetection
{
    // Detection
    TargetDetection detection;

    // Timestamp, in second
    double t = 0.;

    StampedTargetDetection(const TargetDetection &detection, double t)
        : detection(detection), t(t)
    {
    }

    // for convenience
    const auto &cornerIds() const { return detection.cornerIds; }
    const auto &corners() const { return detection.corners; }
    const auto &detected() const { return detection.detected; }
    const auto &cornerCount() const { return detection.count; }
    auto valid() const { return detection.valid(); }
    auto complete() const { return detection.complete(); }
    auto total() const { return detection.total(); }
};

using StampedTargetDetections = std::vector<StampedTargetDetection>;

} // namespace tl
