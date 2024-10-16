#include "StereoModuleTask.h"

#include <chrono>

#include <tCalibration/CalibrationIO>
#include <tCalibration/CameraImuCalibration>
#include <tCalibration/CameraIntrinsicsCalibration>
#include <tCalibration/CameraPoseCalibration>
#include <tCalibration/InertialBasedScaleEstimation>
#include <tCalibration/SplineTypes>
#include <tCalibration/StereoCameraCalibration>
#include <tCalibration/StereoRectify>
#include <tCamera/OmnidirectionalCameraModel>
#include <tCamera/Types>
#include <tCore/Math>
#include <tCore/TimeUtils>
#include <tMath/Eigen/Utils>
#include <tMotion/ImuIntrinsics>
#include <tMotion/ImuNoise>
#include <tMotion/SplineErrorWeighting>
#include <tVision/BlurDetection>
#include <tVision/Target/KalibrAprilTagBoard>

#include "StereoModuleData.h"

namespace tl {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {
constexpr size_t kEstimatedViewCount{700};
constexpr size_t kEstimatedMotionCount{1000};
} // namespace

///------- StereoModuleTask::Impl starts from here
class StereoModuleTask::Impl
{
public:
    Impl();

    void init(const Options& options);

    // They are designed to run in order.
    bool detectCorners(const StereoImageData& data, bool keep);
    bool calibCameraIntrinsics();
    void calibStereoExtrinsics();
    void calibCameraPoses();
    bool calibCameraImuTransform();
    void calcStereoRectifyMap();

    void clearData();

public:
    Options m_opts;
    TaskState m_state{TaskState::None};

    std::string m_uuid{};
    StereoModuleInfo m_deviceInfo{};
    std::shared_ptr<CalibBoardBase> m_board;
    std::unique_ptr<CameraIntrinsicsCalibration> m_calibCamIntri;
    std::unique_ptr<CameraPoseCalibration> m_calibCamPose;
    StampedTargetDetections m_leftDetections, m_rightDetections;
    ImuDatas m_imuDatas;
    std::array<Camera, StereoSize> m_stereo;
    std::array<double, StereoSize> m_rpe;
    Eigen::Matrix3d m_imuToLeftRotation, m_imuToRightRotation;
    Eigen::Quaterniond m_q_l_r;
    Eigen::Vector3d m_t_l_r;
    int m_viewCount{0};
    int m_intriViewCount{0};
    ResultSummary::Error m_verifyError{ResultSummary::Error::NotReady};
};

StereoModuleTask::Impl::Impl() {}

void StereoModuleTask::Impl::init(const Options& options)
{
    //
    m_opts = options;
}

bool StereoModuleTask::Impl::detectCorners(const StereoImageData& stereo,
                                           bool keep)
{
    const auto& timestamp = stereo.timestamp;
    const auto leftDetection = m_board->computeObservation(stereo.left);
    const auto rightDetection = m_board->computeObservation(stereo.right);

    if (!leftDetection.valid() || !rightDetection.valid()) {
        LOG(INFO) << "Skip view "
                     "("
                  << timestamp
                  << "): "
                     "Failed to detect corners on both views.";
        return false;
    }

    LOG(INFO) << "Detected left/right corners: " << leftDetection.count << "/"
              << rightDetection.count;

    m_leftDetections.emplace_back(leftDetection, timestamp);
    m_rightDetections.emplace_back(rightDetection, timestamp);

    if (keep) {
        m_calibCamIntri->addDetection(m_leftDetections.back(), kCameraLeftId);
        m_calibCamIntri->addDetection(m_rightDetections.back(), kCameraRightId);
    }

    return true;
}

bool StereoModuleTask::Impl::calibCameraIntrinsics()
{
    const auto leftSummary = m_calibCamIntri->calibrate(kCameraLeftId);
    const auto rightSummary = m_calibCamIntri->calibrate(kCameraRightId);
    const bool success = leftSummary.success() && rightSummary.success();

    if (!success) {
        return false;
    }

    const auto* left = m_calibCamIntri->camera(kCameraLeftId);
    const auto* right = m_calibCamIntri->camera(kCameraRightId);

    LOG(INFO) << "\n"
                 "******* "
                 "Calibrated camera intrinsics"
                 " *******"
                 "\n"
                 "Left camera: "
                 "\n"
              << *left->cameraIntrinsics()
              << "\n---\n"
                 "Right camera: "
                 "\n"
              << *right->cameraIntrinsics();

    m_stereo[kCameraLeftId] = *left;
    m_stereo[kCameraRightId] = *right;
    m_rpe[kCameraLeftId] = leftSummary.finalRPE;
    m_rpe[kCameraRightId] = rightSummary.finalRPE;

    return true;
}

void StereoModuleTask::Impl::calibStereoExtrinsics()
{
    auto scene = m_calibCamIntri->scene();
    scene->pairCamera(kCameraLeftId, kCameraRightId);

    StereoCameraCalibration calibStereo;
    calibStereo.setScene(scene);

    if (!calibStereo.calibrate()) {
        LOG(ERROR) << "Failed to calibrate stereo camera extrinsics.";
        return;
    }

    calibStereo.getTransform(m_q_l_r, m_t_l_r);

    // NOTE: This is correct before they share scene
    const auto* left = m_calibCamIntri->camera(kCameraLeftId);
    const auto* right = m_calibCamIntri->camera(kCameraRightId);

    LOG(INFO) << "\n"
                 "******* "
                 "Camera intrinsics after stereo calibration"
                 " *******"
                 "\n"
                 "Left camera: "
                 "\n"
              << *left->cameraIntrinsics()
              << "\n---\n"
                 "Right camera: "
                 "\n"
              << *right->cameraIntrinsics();

    double roll, pitch, yaw;
    math::RotationMatrixToRPY(m_q_l_r.toRotationMatrix(), roll, pitch, yaw);
    LOG(INFO) << "******* "
                 "Stereo extrinsics"
                 " *******"
                 "\n"
                 "Orientation (roll, pitch, yaw): "
              << roll << ", " << pitch << ", " << yaw
              << "\n"
                 "Translation: "
              << m_t_l_r.transpose();
}

void StereoModuleTask::Impl::calibCameraPoses()
{
    m_calibCamPose->calibrate(m_leftDetections, m_stereo[kCameraLeftId],
                              kCameraLeftId);
    m_calibCamPose->calibrate(m_rightDetections, m_stereo[kCameraRightId],
                              kCameraRightId);

    constexpr bool kOptimizeBoardPoints = false;
    if constexpr (kOptimizeBoardPoints) {
        m_calibCamPose->optimizeBoardPoints();
        m_calibCamPose->optimizeAllPoses();
    }
    m_calibCamPose->filterBadPoses();
}

bool StereoModuleTask::Impl::calibCameraImuTransform()
{
    ///  Camera-IMU rotation estimation
    const auto scene = m_calibCamPose->scene();
    double visStart{kMaxDouble}, visEnd{kMinDouble};
    for (const auto& camId : {kCameraLeftId, kCameraRightId}) {
        for (const auto& viewId : scene->sharedCameraViewIds(camId)) {
            const auto time = scene->view(viewId)->timestamp();
            visStart = std::min(visStart, time);
            visEnd = std::max(visEnd, time);
        }
    }

    Timeline acc_times;
    std::vector<Vector3d> linear_accs;
    for (const auto& acc : m_imuDatas.acc.d()) {
        if (acc.timestamp() > visEnd || acc.timestamp() < visStart) {
            continue;
        }

        acc_times.push_back(acc.timestamp());
        linear_accs.push_back(acc.asVector());
    }

    Timeline gyr_times;
    std::vector<Vector3d> ang_vels;
    for (const auto& gyro : m_imuDatas.gyro.d()) {
        if (gyro.timestamp() > visEnd || gyro.timestamp() < visStart) {
            continue;
        }

        gyr_times.push_back(gyro.timestamp());
        ang_vels.push_back(gyro.asVector());
    }

    InertialBasedScaleEstimation ibse;

    if (!ibse.setInertialData(acc_times, linear_accs, gyr_times, ang_vels)) {
        LOG(ERROR) << "Failed to estimate visual scale: "
                      "Invalid input inertial data. ";
        return false;
    }

    // NOTE: We use the byproduct (rotation) other than scale
    auto estVisualScale = [&ibse, this](CameraId camId, const std::string& hint,
                                        Eigen::Matrix3d& rotation) -> bool {
        Timeline vis_times;
        std::vector<Vector3d> vis_poses;
        std::vector<Quaterniond> vis_rots;
        const auto scene = m_calibCamPose->scene();
        for (const auto& viewId : scene->sharedCameraViewIds(camId)) {
            const auto* view = scene->view(viewId);
            const auto& camera = view->camera();
            const auto time = view->timestamp();
            const Quaterniond q{camera.orientationAsRotationMatrix()};
            vis_times.push_back(time);
            vis_poses.push_back(camera.position());
            vis_rots.push_back(q);
        }

        if (!ibse.setVisualData(vis_times, vis_poses, vis_rots)) {
            return false;
        }

        LOG(INFO) << "======= "
                     "Start Estimating "
                  << hint
                  << " scale"
                     " =======";
        const auto errCode = ibse.initialAlignmentEstimation();
        if (errCode != InertialBasedScaleEstimation::Error::None) {
            LOG(ERROR) << "Failed to estimate " << hint << " scale.";
            return false;
        }

        rotation = ibse.imuToCameraRotation();

        double r, p, y;
        math::RotationMatrixToRPY(ibse.imuToCameraRotation(), r, p, y);
        LOG(INFO) << "IMU to " << hint
                  << " rotation is (roll, pitch, yaw): " << r << ", " << p
                  << ", " << y;
        return true;
    };

    const auto success =
        estVisualScale(kCameraLeftId, "left camera", m_imuToLeftRotation) &&
        estVisualScale(kCameraRightId, "right camera", m_imuToRightRotation);

    if (!success) {
        LOG(ERROR) << "Failed to estimate visual scale.";
    }

    return success;

    /// Full optimization, WARNING: Not tested!
    constexpr double kAccQuality = 0.98;
    constexpr double kGyrQuality = 0.96;
    constexpr double kMinAccSpacing = 0.01;
    constexpr double kMaxAccSpacing = 0.15;
    constexpr double kMinGyrSpacing = 0.01;
    constexpr double kMaxGyrSpacing = 0.2;

    SplineErrorWeighting sew;
    SplineWeightingData sewData;
    sew.estKnotSpacingAndVariance(m_imuDatas.acc, kAccQuality, kMinAccSpacing,
                                  kMaxAccSpacing, sewData.r3_dt,
                                  sewData.r3_var);
    sew.estKnotSpacingAndVariance(m_imuDatas.gyro, kGyrQuality, kMinGyrSpacing,
                                  kMaxGyrSpacing, sewData.so3_dt,
                                  sewData.so3_var);

    constexpr double kInitRSLineDelay = 0.;
    constexpr int kOptimizeSplineIteration = 50;
    constexpr int kOptimizeLdIteration = 10;
    constexpr bool kReestimatebiases = true;
    constexpr bool kCalibrateLineDelay = false;
    constexpr bool kGlobalShutter = false;

    const Sophus::SE3d T_i_c_init{Quaterniond{m_imuToLeftRotation}.conjugate(),
                                  Vector3d::Zero()};

    CameraImuCalibration calibCamImu;

    ImuIntrinsics accIntri;
    ImuIntrinsics gyrIntri;
    calibCamImu.initSpline(*m_calibCamPose->scene().get(), T_i_c_init, sewData,
                           ibse.imuToCameraTimeOffset(), m_imuDatas,
                           kInitRSLineDelay, accIntri, gyrIntri);

    const int gravityAxis = 2; // Z
    int flags = SplineOptimizeType::SPLINE | SplineOptimizeType::T_I_C;
    if constexpr (kReestimatebiases) {
        flags |= SplineOptimizeType::IMU_BIASES;
    }

    if (gravityAxis != -1) {
        Vector3d gravity(0, 0, 0);
        gravity[gravityAxis] = 9.81;
        calibCamImu.setKnownGravityDir(gravity);
    }
    else {
        flags |= SplineOptimizeType::GRAVITY_DIR;
    }

    double rpe = calibCamImu.optimize(kOptimizeSplineIteration, flags);

    double rpe_after_ld = rpe;
    if constexpr (kCalibrateLineDelay && !kGlobalShutter) {
        flags = SplineOptimizeType::CAM_LINE_DELAY;
        rpe_after_ld = calibCamImu.optimize(kOptimizeLdIteration, flags);
    }

    // Results
    LOG(INFO) << "Mean reprojection error: " << rpe
              << "px"
                 "\n"
                 "Mean reprojection error after line delay optimization: "
              << rpe_after_ld
              << "px"
                 "\n"
                 "Optimized gravity: "
              << calibCamImu.estimatedGravity().transpose()
              << "\n"
                 "Accelerator bias at time 0: "
              << calibCamImu.acceleratorBiasAt(0).transpose()
              << "\n"
                 "Gyroscope at time 0: "
              << calibCamImu.gyroscopeBiasAt(0).transpose();

    Quaterniond q_i_c;
    Vector3d t_i_c;
    calibCamImu.imuToCameraTransform(q_i_c, t_i_c);

    const double calib_line_delay =
        time::sToUs(calibCamImu.calibratedRSLineDelay());

    LOG(INFO) << "IMU to camera transform: "
                 "\n"
                 "Rotation: "
                 "\n"
                 "In quaternion(w, x, y, z): "
              << q_i_c.coeffs()
              << "\n"
                 "In rotation matrix: "
                 "\n"
              << q_i_c.matrix()
              << "\n"
                 "Translation: "
              << t_i_c.transpose()
              << "\n"
                 "Initialized line delay [us]: "
              << time::sToNs(kInitRSLineDelay)
              << "\n"
                 "Calibrated line delay [us]: "
              << calib_line_delay;
}

void StereoModuleTask::Impl::calcStereoRectifyMap()
{
    // TODO
}

void StereoModuleTask::Impl::clearData()
{
    m_leftDetections.clear();
    m_leftDetections.reserve(kEstimatedViewCount);
    m_rightDetections.clear();
    m_rightDetections.reserve(kEstimatedViewCount);

    m_imuDatas.clear();
    m_imuDatas.reserve(kEstimatedMotionCount);

    m_viewCount = 0;

    m_imuToLeftRotation = Matrix3d::Identity();
    m_imuToRightRotation = Matrix3d::Identity();
    m_q_l_r = Quaterniond::Identity();
    m_t_l_r = Vector3d::Zero();

    m_calibCamIntri->clearAllViews(kCameraLeftId);
    m_calibCamIntri->clearAllViews(kCameraRightId);

    // TODO: Clear camera info (?)
}

///------- StereoModuleTask starts from here
StereoModuleTask::StereoModuleTask(const Options& options)
    : d(std::make_unique<Impl>())
{
    d->init(options);
}

StereoModuleTask::~StereoModuleTask() = default;

void StereoModuleTask::setUuid(const std::string& uuid)
{
    if (uuid.empty()) {
        return;
    }

    d->m_uuid = uuid;
}

const std::string& StereoModuleTask::uuid() const { return d->m_uuid; }

void StereoModuleTask::setDeviceInfo(const StereoModuleInfo& info)
{
    d->m_deviceInfo = info;
}

const StereoModuleInfo& StereoModuleTask::devicId() const
{
    return d->m_deviceInfo;
}

void StereoModuleTask::prepare()
{
    // 1. Create board
    // d->m_board.reset(new KalibrAprilTagBoard(6, 6, 0.14, 0.3)); // 1.2m foam
    // d->m_board.reset(new KalibrAprilTagBoard(6, 6, 0.0352, 0.3)); // small
    // FIXME: Use default parameters to pass build
    d->m_board.reset(new KalibrAprilTagBoard()); // 1m flim
    // d->m_board.reset(new CheckerBoard(8, 11, 0.03, 0.03)); // glass

    // 2. Create Calibration interface
    {
        CameraIntrinsicsCalibration::Options opts;
        opts.intrinsicsType = CameraIntrinsicsType::Omnidirectional;
        opts.imageHeight = 480;
        opts.imageWidth = 640;
        opts.checkSampleDistance = false;
        opts.sampleDistance = 0.02;
        opts.checkDetectionCoverage = false;
        opts.centerRatio = 0.7;
        opts.optimizeBoardPoints = false;

        d->m_calibCamIntri.reset(new CameraIntrinsicsCalibration(opts));
        d->m_calibCamIntri->setupScene(d->m_board);
    }

    {
        CameraPoseCalibration::Options opts;
        //

        d->m_calibCamPose.reset(new CameraPoseCalibration(opts));
        d->m_calibCamPose->setupScene(*d->m_board.get());
    }

    if (d->m_state > TaskState::Initialized) {
        d->clearData();
    }

    d->m_state = TaskState::Initialized;
}

StereoModuleTask::TaskState StereoModuleTask::state() const
{
    return d->m_state;
}

void StereoModuleTask::addStereoData(const StereoImageData& data)
{
    if (d->m_viewCount >= d->m_opts.maxViewCount) {
        return;
    }

    const bool keep = ((d->m_viewCount % d->m_opts.skipStep) == 0) &&
                      (d->m_intriViewCount < d->m_opts.maxIntrinsicsViewCount);
    const auto success = d->detectCorners(data, keep);
    if (success) {
        ++d->m_viewCount;
        if (keep) {
            ++d->m_intriViewCount;
        }

        d->m_state = TaskState::DataCollected;
    }
}

void StereoModuleTask::addMotionData(const ImuData& data)
{
    d->m_imuDatas.push_back(data);
}

void StereoModuleTask::setMotionData(const ImuDatas& data)
{
    d->m_imuDatas = data;
}

void StereoModuleTask::clearData()
{
    d->clearData();
    d->m_state = std::min(d->m_state, TaskState::Initialized);
}

bool StereoModuleTask::startCalculation()
{
    if (d->m_viewCount < d->m_opts.minViewCount ||
        d->m_intriViewCount < d->m_opts.minIntrinsicsViewCount) {
        LOG(WARNING) << "Not enough view samples to start calculation. "
                        "Valid view pairs: "
                     << d->m_viewCount
                     << ". "
                        "Valid intrinsics view pairs: "
                     << d->m_intriViewCount;
        return false;
    }

    LOG(INFO) << "Left/Right detection count: " << d->m_leftDetections.size()
              << "/" << d->m_rightDetections.size()
              << "\n"
                 "IMU count: "
              << d->m_imuDatas.size();

    if (!d->calibCameraIntrinsics()) {
        LOG(ERROR) << "Failed to continue calibration: "
                      "Camera intrinsics calibration failed.";
        return false;
    }

    d->calibCameraPoses();
    if (!d->calibCameraImuTransform()) {
        LOG(ERROR) << "Failed to continue calibration: "
                      "Camera-Imu Rotation calibration failed.";
        return false;
    }

    d->calibStereoExtrinsics();

    d->m_state = TaskState::Calibrated;
    return true;
}

StereoModuleTask::ResultSummary StereoModuleTask::verifyCalibResults(
    const ResultReference& ref) const
{
    using Error = StereoModuleTask::ResultSummary::Error;

    ResultSummary summary;

    if (!readyToVerify()) {
        summary.error = Error::NotReady;
        LOG(INFO) << "Not ready to verify results.";
        return summary;
    }

    /// Optimization
    // RPE, TODO: 1. Which RPE is useful?
    summary.leftRPE = d->m_rpe[kCameraLeftId];
    summary.rightRPE = d->m_rpe[kCameraRightId];

    if (!std::isless(d->m_rpe[kCameraLeftId], ref.maxRPE)) {
        summary.error |= Error::PoorLeftRPE;
    }
    if (!std::isless(d->m_rpe[kCameraRightId], ref.maxRPE)) {
        summary.error |= Error::PoorRightRPE;
    }

    /// Sensors
    const auto& left = d->m_stereo[kCameraLeftId];
    const auto& right = d->m_stereo[kCameraRightId];

    // Focal length, TODO: 1. Which camera model?
    summary.leftFocalLength = left.focalLength();
    summary.rightFocalLength = right.focalLength();

    // Principal points
    auto checkPrincipalPoint = [&ref](const Vector2d& oc) {
        return math::inSymmetricRange(oc.x(), ref.expectedPrincipalPointX,
                                      ref.principalPointXTolerance) &&
               math::inSymmetricRange(oc.y(), ref.expectedPrincipalPointY,
                                      ref.principalPointYTolerance);
    };

    const Vector2d leftOC{left.principalPointX(), left.principalPointY()};
    const Vector2d rightOC{right.principalPointX(), right.principalPointY()};
    const Vector2d ocOffset = leftOC - rightOC;

    summary.leftCx = left.principalPointX();
    summary.leftCy = left.principalPointY();
    summary.rightCx = right.principalPointX();
    summary.rightCy = right.principalPointY();
    summary.diffCx = ocOffset.x();
    summary.diffCy = ocOffset.y();

    if (!checkPrincipalPoint(leftOC)) {
        summary.error |= Error::PoorLeftPrincipalPoint;
    }
    if (!checkPrincipalPoint(rightOC)) {
        summary.error |= Error::PoorRightPrincipalPoint;
    }
    // Now check y only
    if (!std::isless(std::abs(ocOffset.y()), ref.principalPointDiffTolerance)) {
        summary.error |= Error::PoorPrincipalPointConsistency;
    }

    /// Stereo
    // Baseline
    const auto baseline = d->m_t_l_r.norm();
    summary.baseline = baseline;
    if (!math::inSymmetricRange(baseline, ref.expectedBaseline,
                                ref.baselineTolerance)) {
        summary.error |= Error::PoorBaseline;
    }

    // Inter camera Rotation
    const AngleAxisd aa{d->m_q_l_r};
    summary.interCameraRotation = math::radToDeg(aa.angle());
    double roll{0.}, pitch{0.}, yaw{0.};
    math::QuaternionToRPY(d->m_q_l_r, roll, pitch, yaw);
    summary.interCameraRoatationRPY = {
        math::radToDeg(roll), math::radToDeg(pitch), math::radToDeg(yaw)};
    if (!math::inSymmetricRange(summary.interCameraRotation,
                                ref.expectedInterCameraRotation,
                                ref.interCameraRotationTolerance)) {
        summary.error |= Error::PoorInterCameraRotation;
    }

    // Imu-Camera rotation
    const AngleAxisd aa_imuToLeft{d->m_imuToLeftRotation};
    const AngleAxisd aa_imuToRight{d->m_imuToRightRotation};
    summary.imuLeftCameraRotation = math::radToDeg(aa_imuToLeft.angle());
    summary.imuRightCameraRotation = math::radToDeg(aa_imuToRight.angle());
    if (!math::inSymmetricRange(summary.imuLeftCameraRotation,
                                ref.expectedImuCameraRotation,
                                ref.imuCameraRotationTolerance) ||
        !math::inSymmetricRange(summary.imuRightCameraRotation,
                                ref.expectedImuCameraRotation,
                                ref.imuCameraRotationTolerance)) {
        summary.error |= Error::PoorImuCameraRotation;
    }

    // Change task state
    d->m_state = TaskState::Verified;
    d->m_verifyError = summary.error;

    return summary;
}

StereoModuleTask::ResultSummary::Error StereoModuleTask::notAppliedErrors()
{
    using Error = StereoModuleTask::ResultSummary::Error;
    return Error::PoorLeftRPE | Error::PoorRightRPE |
           Error::PoorLeftFocalLength | Error::PoorRightFocalLength |
           Error::PoorLeftPrincipalPoint | Error::PoorRightPrincipalPoint |
           Error::PoorInterCameraRotationInEuler;
}

bool StereoModuleTask::pass() const
{
    // FIXME: Equivalent logic???
    const auto appliedError =
        (d->m_verifyError | notAppliedErrors()) ^ notAppliedErrors();
    return d->m_state == TaskState::Verified &&
           appliedError == ResultSummary::NoError;
}

bool StereoModuleTask::readyToCalculate() const
{
    return d->m_state >= TaskState::DataCollected;
}

bool StereoModuleTask::readyToVerify() const
{
    return d->m_state >= TaskState::Calibrated;
}

bool StereoModuleTask::readyToUpload() const
{
    return d->m_state >= TaskState::Verified;
}

std::string StereoModuleTask::toVioConfigFile() const
{
    io::CalibMetaData meta;
    meta.uuid = d->m_uuid;
    meta.calibDateTime = []() -> std::string {
        const auto now = std::chrono::current_zone()->to_local(
            std::chrono::system_clock::now());
        return std::format("{:%Y-%m-%d %X}", now);
    }();
    meta.hardwareAddr = d->m_deviceInfo.hardwareAddr;
    meta.deviceSN = d->m_deviceInfo.sn;

    const auto& left = d->m_stereo[kCameraLeftId];
    const auto& right = d->m_stereo[kCameraRightId];

    const Matrix3d i_R_left = d->m_imuToLeftRotation.transpose();
    const Vector3d magic_i_t_left{-0.01383601, 0.04246329, -0.01212596};
    Matrix4d i_T_left = Matrix4d::Identity();
    i_T_left.block<3, 3>(0, 0) = i_R_left;
    i_T_left.block<3, 1>(0, 3) = magic_i_t_left;

    const Matrix3d i_R_right =
        i_R_left * d->m_q_l_r.toRotationMatrix().transpose();
    const Vector3d i_t_right = magic_i_t_left - i_R_right * d->m_t_l_r;
    Matrix4d i_T_right = Matrix4d::Identity();
    i_T_right.block<3, 3>(0, 0) = i_R_right;
    i_T_right.block<3, 1>(0, 3) = i_t_right;

    return io::toVioYamlString(left, right, i_T_left, i_T_right, meta);
}

std::tuple<std::string, std::string> StereoModuleTask::toStereoMaps() const
{
    const auto& left = d->m_stereo[kCameraLeftId];
    const auto& right = d->m_stereo[kCameraRightId];

    // TODO: Keep two magic values consistent
    const Matrix3d R_i_l = d->m_imuToLeftRotation.transpose();
    const Vector3d t_i_l{-0.01383601, 0.04246329, -0.01212596};
    const Matrix3d R_i_r = R_i_l * d->m_q_l_r.toRotationMatrix().transpose();
    const Vector3d t_i_r = t_i_l - R_i_r * d->m_t_l_r;

    const Matrix3d R_l_r = R_i_l.transpose() * R_i_r;
    const Vector3d t_l_r = R_i_l.transpose() * (t_i_r - t_i_l);
    const Matrix3d R_r_l = R_i_r.transpose() * R_i_l;
    const Vector3d t_r_l = R_i_r.transpose() * (t_i_l - t_i_r);

    cv::Mat map1_left, map2_left, map1_right, map2_right;
    const bool success =
        initUndistortRectifyMap(left, right, R_r_l, t_r_l, map1_left, map2_left,
                                map1_right, map2_right);
    if (!success) {
        return {};
    }

    std::string left_str, right_str;
    io::toStereoUndistortRectifyMapString(map1_left, map2_left, map1_right,
                                          map2_right, left_str, right_str);

    return std::make_tuple(left_str, right_str);
}

} // namespace tl
