#pragma once

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <QCoreApplication>

#include <tCalib/MCMB/Calibration>

#include "ReportSummary.h"

namespace tl {

struct DeviceInfo;
struct ImuData;
struct ImuDatas;
struct ProductInfo;
struct StereoImages;
struct TaskInfo;

struct StereoImageInfo
{
    std::string left, right;
};

class StereoCameraCalibrationTask
{
    Q_DECLARE_TR_FUNCTIONS(tl::StereoCameraCalibrationTask)

public:
    struct Options
    {
        /////// Task overall

        // Max number of total stereo views.
        int maxNumViews{700};

        // Min number of total stereo views.
        int minNumViews{300};

        // It's not neccessary to use all the incoming stereo images to
        // calibrate camera intrinsics and extrinsics.
        int skipStep{6};

        /////// Board detection

        // Board type to use in calibration
        // TODO: Setup target options
        // 0. AprilTagOfficial; 1. AprilTagKalibr; 2. ArUco

        // Refine detected corners after board detection.
        // WARNING: Not stable yet, turn off by default.
        bool refineCorners{false};

        mcmb::Calibration::Options mcmbOptions;

        Options() {}
    };

    using Ptr = std::unique_ptr<StereoCameraCalibrationTask>;
    using ConstPtr = std::unique_ptr<const StereoCameraCalibrationTask>;

    explicit StereoCameraCalibrationTask(const Options& options = {});
    ~StereoCameraCalibrationTask();

    /// Properties
    const TaskInfo& info() const;
    void resetTaskInfo();
    const DeviceInfo& deviceInfo() const;
    void setDeviceInfo(const DeviceInfo& info);
    const Options& options() const;
    void setOptions(const Options& opts);
    bool setFromJson(const std::string& json);
    bool setTargetFromJson(const std::string& json);
    void setProductInfo(const ProductInfo& info);

    /// Data
    void addStereoData(const StereoImages& stereo,
                       const StereoImageInfo& info = {});
    // Incremental interface
    void addMotionData(const ImuData& imu);
    // Batch interface
    void setMotionData(const ImuDatas& imu);
    // Keep settings and configs, clear cached data
    void clearData();

    /// Actions
    enum Error
    {
        None = 0,

        NotEnoughViews,
        FailedCameraCalibration,
        FailedCameraImuCalibration
    };
    Error startCalibration();
    bool calibrated() const;

    struct Reference
    {
        double maxRpe = 0.5;

        double focalLengthTolerance = 10.;

        std::array<double, 2> principalPointTolerance = {5., 5.};

        std::array<double, 2> maxPrincipalPointDiff = {20., 20.};

        double baselineTolerance = 0.03;

        double interCameraRotationTolerance = 0.02;

        double imuCameraRotationTolerance = 0.02;

        double maxImuCameraTimeOffset = 20.;
    };
    ReportSummary resultAsSummary(const Reference& ref) const;
    std::string resultAsYaml() const;

    // TEST
    bool saveDetailResultTo(const std::string& root) const;

private:
    class Impl;
    const std::unique_ptr<Impl> d;
};

} // namespace tl

namespace nlohmann {

void to_json(json& j, const tl::StereoCameraCalibrationTask::Options& opts);
void from_json(const json& j, tl::StereoCameraCalibrationTask::Options& opts);

void to_json(json& j, const tl::StereoCameraCalibrationTask::Reference& ref);
void from_json(const json& j, tl::StereoCameraCalibrationTask::Reference& ref);

} // namespace nlohmann
