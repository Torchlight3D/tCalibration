#pragma once

#include <QCoreApplication>
#include <yaml-cpp/yaml.h>

#include <json/json.hpp>

#include <tCalib/MCMB/Calibration>

#include "ReportSummary.h"

namespace tl {

struct DeviceInfo;
struct ImuData;
struct ImuDatas;
struct MonoImage;
struct ProductInfo;
struct TaskInfo;

// FIXME:
// 1. Most of the codes are copied from StereoCameraCalibrationTask, is it
// possible to abstract the pattern?
class MonoCameraCalibrationTask
{
    Q_DECLARE_TR_FUNCTIONS(tl::MonoCameraCalibrationTask)

public:
    struct Options
    {
        // Max number of total views.
        int maxNumViews{700};

        // Min number of total views.
        int minNumViews{300};

        // It's not neccessary to use all the incoming images to calibrate
        // camera intrinsics.
        int skipStep{6};

        mcmb::Calibration::Options mcmbOptions;

        Options() {}
    };

    using Ptr = std::unique_ptr<MonoCameraCalibrationTask>;
    using ConstPtr = std::unique_ptr<const MonoCameraCalibrationTask>;

    explicit MonoCameraCalibrationTask(const Options& options = {});
    ~MonoCameraCalibrationTask();

    /// Properties
    const TaskInfo& info() const;
    void resetTaskInfo();
    const DeviceInfo& deviceInfo() const;
    void setDeviceInfo(const DeviceInfo& info);
    const Options& options() const;
    void setOptions(const Options& options);
    bool setFromJson(const std::string& json);
    bool setTargetFromJson(const std::string& json);
    void setProductInfo(const ProductInfo& info);

    /// Data
    void addMonoData(const MonoImage& data, const std::string& imgPath = {});
    // Incremental interface
    void addMotionData(const ImuData& data);
    // Batch interface
    void setMotionData(const ImuDatas& data);
    void clearData();

    /// Actions
    enum Error
    {
        None = 0,

        NotEnoughViews,
        FailedCalibration,
        FailedCameraImuCalibration
    };
    Error startCalibration();
    bool calibrated() const;

    struct Reference
    {
        double maxRpe = 0.5;

        double focalLengthTolerance = 10.;

        std::array<double, 2> principalPointTolerance = {5., 5.};

        double imuCameraRotationTolerance = 0.02;

        double maxImuCameraTimeOffset = 20.;
    };

    ReportSummary resultAsSummary(const Reference& ref) const;
    std::string resultAsYaml() const;

private:
    class Impl;
    const std::unique_ptr<Impl> d;
};

} // namespace tl

namespace nlohmann {

void to_json(json& j, const tl::MonoCameraCalibrationTask::Options& opts);
void from_json(const json& j, tl::MonoCameraCalibrationTask::Options& opts);

void to_json(json& j, const tl::MonoCameraCalibrationTask::Reference& ref);
void from_json(const json& j, tl::MonoCameraCalibrationTask::Reference& ref);

} // namespace nlohmann
