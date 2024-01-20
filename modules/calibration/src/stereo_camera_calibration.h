#pragma once

#include <memory>

#include <tCalibration/CameraIntrinsicsCalibration>
#include <tTarget/CalibBoardBase>
#include <tMvs/Scene>

namespace tl {

class StereoCameraCalibration
{
public:
    struct Options
    {
        Options() {}
    };

    explicit StereoCameraCalibration(const Options& options = {});
    ~StereoCameraCalibration();

    /// Properties
    const Options& options() const;
    Options& rOptions();

    /// Data
    // Calibrated scene interface. Make sure:
    // 1. At least two different cameras (Id) in the scene.
    // 2. They are calibrated.
    void setScene(const Scene::Ptr scene);
    // Empty scene interface. This method will clear the scene (if there's any)
    // WARNING: Not ready yet
    void setupScene(const CalibBoardBase& board);

    /// Actions
    // Calibrated scene interface.
    bool calibrate();
    // Empty scene interface. WARNING: Not ready yet
    bool calibrate(const StampedTargetDetections& left,
                   const StampedTargetDetections& right);

    /// Results
    void getTransform(Eigen::Quaterniond& orientation,
                      Eigen::Vector3d& translation) const;
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
