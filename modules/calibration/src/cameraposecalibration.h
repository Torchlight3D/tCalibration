#pragma once

#include <opencv2/core/types.hpp>

#include <tCamera/Camera>
#include <tMvs/Scene>
#include <tVision/Target/TargetDetection>

namespace tl {

class CalibBoardBase;

class CameraPoseCalibration
{
public:
    // TODO:
    // 1. TrackMinObs;
    // 2. PnP type;
    // 3. MinInliner
    struct Options
    {
        // Minimum number of inliers for pose estimation success
        size_t min_num_points_ = 8;

        // Minimum number of observations of a scene point to be optimized
        size_t min_num_obs_for_optim_ = 30;

        Options() {}
    };

    explicit CameraPoseCalibration(const Options& options = {});
    ~CameraPoseCalibration();

    /// Properties
    const Options& options() const;
    Options& rOptions();

    void setupScene(const CalibBoardBase& board);

    /// Actions
    // Assume the camera is calibrated
    bool calibrate(const StampedTargetDetections& detections,
                   const Camera& camera, CameraId id = {});

    void optimizeAllPoses();
    void optimizeBoardPoints();
    void filterBadPoses();

    /// Results
    Scene::Ptr scene() const;

    /// IO
    bool setFromJson(const std::string& json);
    void toJsonString(std::string& json) const;

private:
    class Impl;
    const std::unique_ptr<Impl> d;

    friend class Impl;
};

} // namespace tl
