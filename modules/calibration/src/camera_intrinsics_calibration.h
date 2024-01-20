#pragma once

#include <string>

#include <tTarget/TargetDetection>
#include <tCamera/CameraIntrinsics>
#include <tMvs/MvsTypes>
#include <tMvs/Scene>

namespace tl {

class CalibBoardBase;
class Camera;

// NOTE:
// 1. Assume all the input images are same size.

// TODO:
// 1. Dont need StampedTargetDetection
// 2. Prepare the scene internally
class CameraIntrinsicsCalibration
{
public:
    struct Options
    {
        // a.k.a. voxel grid size. Use this value to control image data sample
        // rate. If the coming view is too close to any existed view, we drop
        // the new one. In meter
        double sampleDistance = 0.04;
        bool checkSampleDistance = false;

        double centerRatio = 0.7;
        bool checkDetectionCoverage = false;

        // Image size is used to initialize principal point
        int imageWidth = -1;
        int imageHeight = -1;

        // Minimum view to do a valid calibration
        int minViewCount = 10;

        // Camera Model to estimate
        CameraIntrinsics::Type intrinsicsType = CameraIntrinsics::Type::Fisheye;

        // Optimize board points or not. Turn on when calibration board quality
        // is not good.
        bool optimizeBoardPoints = true;

        // Basic validation to ensure some of the fields are not their default
        // values.
        bool isValid() const;

        Options() {}
    };

    explicit CameraIntrinsicsCalibration(const Options& options = {});
    ~CameraIntrinsicsCalibration();

    /// Properties
    void setOptions(const Options& options);
    const Options& options() const;

    /// Data
    void setupScene(std::shared_ptr<CalibBoardBase> board);

    // Incremental interface
    void addDetection(const StampedTargetDetection& detection,
                      CameraId id = {});
    void clearAllViews(CameraId id = {});

    /// Actions

    struct Summary
    {
        double finalRPE{-1.};
        int validViewCount{0};

        enum ErrorCode
        {
            Success,

            /// Options
            InvalidConfigs,

            /// Data
            InvalidCameraId,
            NotEnoughData,
            NotEnoughValidData,

            /// Algo
            PoorObservationCoverage,
            FailedFinalEstimation // When it happens ???
        };
        ErrorCode errCode{InvalidConfigs};

        bool success() const { return errCode == Success; }
    };

    // Incremental interface
    Summary calibrate(CameraId id = {});
    // Batch interface
    Summary calibrate(const StampedTargetDetections& detections,
                      CameraId id = {});

    /// Results
    const Camera* camera(CameraId id = {}) const;
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
