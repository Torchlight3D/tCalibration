#pragma once

#include <memory>

#include <tCore/EnumUtils>
#include <tMotion/ImuData>

namespace tl {

struct StereoImageData;
struct StereoModuleInfo;

class StereoModuleTask
{
public:
    enum class CalibTaskType
    {
        CameraIntrinsics, //!< Calibrate both left and right camera intrinsics
        StereoExtrinsics, //!< Calibrate left to right transform
        CameraContinuousPoses,    //!< Calibrate continuous camera poses
        CameraImuRotation,        //!< Estimate IMU to camera rotation
        CameraImuFullTranslation, //!< Estimate IMU to camera transform
        StereoRectification,      //!< Generate undistortion map
    };

    struct Options
    {
        // We usually use much less data to calibrate camera-only parameters
        // (intrinsics or extrinsics) than motion-involved parameters
        // (Camera-IMU extrinsics)
        int skipStep = 6;
        int maxViewCount = 700;
        int minViewCount = 300;
        int maxIntrinsicsViewCount = 50;
        int minIntrinsicsViewCount = 10;

        // Check the explanation in TaskType
        CalibTaskType taskType = CalibTaskType::CameraIntrinsics;

        // Minimum valid detection pair
        int minDetection = 20;

        Options() {}
    };

    using Ptr = std::unique_ptr<StereoModuleTask>;
    using ConstPtr = std::unique_ptr<const StereoModuleTask>;

    explicit StereoModuleTask(const Options& options = {});
    ~StereoModuleTask();

    /// Properties
    void setUuid(const std::string& uuid);
    const std::string& uuid() const;
    void setDeviceInfo(const StereoModuleInfo& info);
    const StereoModuleInfo& devicId() const;

    // WARNING: Magic initialization inside
    void prepare();

    /// Data
    /// Incremental interfaces
    void addStereoData(const StereoImageData& stereo);
    void addMotionData(const ImuData& data);

    /// Batch interface
    void setMotionData(const ImuDatas& data);

    void clearData();

    /// Actions
    bool startCalculation();

    struct ResultReference
    {
        /// Optimization
        double maxRPE = 0.3; // pixel

        /// Sensors
        // 308 is calculated based on Luba camera sensor specs
        double expectedFocalLength = 308.; // pixel
        double focalLengthTolerance = 10.;
        double expectedPrincipalPointX = 320.; // pixel
        double expectedPrincipalPointY = 240.; // pixel
        double principalPointXTolerance = 5.;
        double principalPointYTolerance = 5.;
        double principalPointDiffTolerance = 20.;

        /// Stereo
        double expectedBaseline = 0.09; // meter
        double baselineTolerance = 0.001;

        // Angle-axis
        double expectedInterCameraRotation = 0.;
        double interCameraRotationTolerance = 0.5; // degree
        double expectedImuCameraRotation = 0.;
        double imuCameraRotationTolerance = 0.5; // degree
    };

    struct ResultSummary
    {
        // TODO: use static_assert to ensure consistentency with Error
        enum Item
        {
            /// Optimization
            LeftRPE,
            RightRPE,
            /// Sensors
            LeftFocalLength,
            RightFocalLength,
            LeftPrincipalPoint,
            RightPrincipalPoint,
            PrincipalPointConsistency,
            /// Stereo
            Baseline,
            InterCameraRotationInAngleAxis,
            InterCameraRotationInEuler,
            ImuCameraRotatation,

            Count,
        };

        /// Optimization
        double leftRPE{-1.};
        double rightRPE{-1.};

        /// Sensors
        double leftFocalLength{1.};
        double rightFocalLength{1.};
        double leftCx{0.}, leftCy{0.};
        double rightCx{0.}, rightCy{0.};
        double diffCx{0.}, diffCy{0.};

        /// Stereo
        double baseline{0.};
        double interCameraRotation{0.};
        double imuLeftCameraRotation{0.};
        double imuRightCameraRotation{0.};
        std::array<double, 3> interCameraRoatationRPY{0., 0., 0.}; // RPY

        enum Error
        {
            NoError = 0x0000,
            NotReady = 0x0001,

            /// Optimization
            PoorLeftRPE = 0x0002,
            PoorRightRPE = 0x0004,

            /// Sensors
            PoorLeftFocalLength = 0x0008,
            PoorRightFocalLength = 0x0010,
            PoorLeftPrincipalPoint = 0x0020,
            PoorRightPrincipalPoint = 0x0040,
            PoorPrincipalPointConsistency = 0x0080,

            /// Stereo
            PoorBaseline = 0x0100,
            PoorInterCameraRotation = 0x0200,
            PoorInterCameraRotationInEuler = 0x400,
            PoorImuCameraRotation = 0x0800,
        };

        // We can't return report in std::string here, because we also need
        // translations
        Error error{Error::NoError};

        bool pass() const { return error == Error::NoError; }
    };

    ResultSummary verifyCalibResults(const ResultReference& reference) const;

    // FIXME: We can't use constexpr here, some problems with MAKE_FLAGS
    static ResultSummary::Error notAppliedErrors();
    bool pass() const;

    /// Results
    // A very simple FSM
    enum class TaskState
    {
        None,
        Initialized,
        DataCollected,
        Calibrated,
        Verified,
    };
    TaskState state() const;

    bool readyToCalculate() const;
    bool readyToVerify() const;
    bool readyToUpload() const;

    std::string toLubaVioResult() const;
    std::tuple<std::string, std::string> toLubaStereoResult() const;

private:
    class Impl;
    const std::unique_ptr<Impl> d;

    friend class Impl;
};

MAKE_FLAGS(StereoModuleTask::ResultSummary::Error)

} // namespace tl
