#include "MonoCameraCalibrationTask.h"

#include <glog/logging.h>

#include <tCalib/CalibrationData>
#include <tCalib/InertialBasedScaleEstimation>
#include <tCalib/IO/CalibrationIO>
#include <tCalib/IO/CameraIO>
#include <tCalib/IO/EigenIO>
#include <tCalib/MCMB/Camera>
#include <tCamera/Types>
#include <tCore/Math>
#include <tCore/TimeUtils>
#include <tMotion/ImuData>
#include <tMath/Eigen/Utils>
#include <tVision/Target/CalibBoardBase>

#include "DeviceInfo.h"
#include "ProductInfo.h"
#include "TaskInfo.h"
#include "version.h"

namespace tl {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

// FIXME: Duplicated code
namespace key {
constexpr char kMaxNumViews[]{"max_num_views"};
constexpr char kMinNumViews[]{"min_num_views"};
constexpr char kSkipStep[]{"skip_step"};
constexpr char kMCMB[]{"mcmb"};

constexpr char kVersion[]{"version"};
constexpr char kDevice[]{"device"};
constexpr char kTask[]{"task"};

constexpr char kCameras[]{"cameras"};
constexpr char kCameraLeft[]{"cam0"};
constexpr char kTd[]{"td"};
constexpr char kCameraToImuTransform[]{"body_T_cam0"};

constexpr char kMaxRpe[]{"max_rpe"};
constexpr char kFocalLengthTolerance[]{"focal_length_tolerance"};
constexpr char kPrincipalPointTolerance[]{"principal_point_tolerance"};
constexpr char kImuCameraRotationTolerance[]{"imu_camera_rotation_tolerance"};
constexpr char kMaxImuCameraTimeOffset[]{"max_imu_camera_time_offset"};
} // namespace key

///------- MonoCameraCalibrationTask::Impl starts from here
class MonoCameraCalibrationTask::Impl
{
public:
    Impl();

    void updateOptions(const Options& options);

public:
    Options _opts;
    DeviceInfo _device;
    ProductInfo _product;
    TaskInfo _info;

    std::shared_ptr<AprilTag> _target;
    std::unique_ptr<mcmb::Calibration> _mcmb;

    // Result
    Eigen::Matrix4d _T_ic0{Eigen::Matrix4d::Identity()};
    double _td_c0i;

    // Cached data
    ImuDatas _imuData;
    int _numViews{0};

    // FIXME: Stupid, dont need this
    bool _calibrated{false};
};

MonoCameraCalibrationTask::Impl::Impl()
    : _mcmb(std::make_unique<mcmb::Calibration>())
{
}

void MonoCameraCalibrationTask::Impl::updateOptions(const Options& opts)
{
    _opts = opts;
    _mcmb->setOptions(opts.mcmbOptions);
}

///------- MonoCameraCalibrationTask starts from here
MonoCameraCalibrationTask::MonoCameraCalibrationTask(const Options& opts)
    : d(std::make_unique<Impl>())
{
    setOptions(opts);
}

MonoCameraCalibrationTask::~MonoCameraCalibrationTask() = default;

const TaskInfo& MonoCameraCalibrationTask::info() const { return d->_info; }

void MonoCameraCalibrationTask::resetTaskInfo() { d->_info = TaskInfo{}; }

const DeviceInfo& MonoCameraCalibrationTask::deviceInfo() const
{
    return d->_device;
}

void MonoCameraCalibrationTask::setDeviceInfo(const DeviceInfo& info)
{
    d->_device = info;
}

const MonoCameraCalibrationTask::Options& MonoCameraCalibrationTask::options()
    const
{
    return d->_opts;
}

void MonoCameraCalibrationTask::setOptions(const Options& opts)
{
    d->updateOptions(opts);
}

bool MonoCameraCalibrationTask::setFromJson(const std::string& json)
{
    try {
        const auto j = nlohmann::json::parse(json);

        Options opts;
        j.get_to(opts);
        setOptions(opts);
    }
    catch (const nlohmann::detail::parse_error& e) {
        LOG(WARNING) << "Failed to parse mono camera calibration task json: "
                     << e.what();
        return false;
    }
    catch (const nlohmann::detail::out_of_range& e) {
        LOG(WARNING)
            << "Incompatiable mono camera calibration task options json: "
            << e.what();
        return false;
    }

    return true;
}

bool MonoCameraCalibrationTask::setTargetFromJson(const std::string& json)
{
    try {
        if (auto target = AprilTag::fromJson(json)) {
            d->_target = std::move(target);
        }
    }
    catch (const nlohmann::detail::parse_error& e) {
        LOG(WARNING) << "Failed to set up tag detector from JSON: " << e.what();
        return false;
    }
    catch (const nlohmann::detail::out_of_range& e) {
        LOG(WARNING) << "Incompatiable tag detector settings: " << e.what();
        return false;
    }

    return true;
}

void MonoCameraCalibrationTask::setProductInfo(const ProductInfo& info)
{
    d->_product = info;
}

void MonoCameraCalibrationTask::addMonoData(const MonoImage& mono,
                                            const std::string& imgPath)
{
    if (!d->_target) {
        return;
    }

    if (!mono.isValid()) {
        return;
    }

    if (d->_numViews > d->_opts.maxNumViews) {
        return;
    }

    const auto& timestamp = mono.timestamp;

    const auto detections =
        d->_target->detectBoards(mono.image, d->_opts.mcmbOptions.boardOptions);

    if (detections.empty()) {
        LOG(WARNING) << "Skip view "
                        "("
                     << timestamp
                     << "): "
                        "Failed to detect corners in view.";
        return;
    }

    // LOG(INFO) << "Detected corners count: " << detection.count;

    if ((d->_numViews % d->_opts.skipStep) == 0) {
        d->_mcmb->addBoardDetections(d->_numViews, kCameraLeftId, detections,
                                     timestamp, imgPath);
    }

    ++d->_numViews;
}

void MonoCameraCalibrationTask::addMotionData(const ImuData& data)
{
    d->_imuData.push_back(data);
}

void MonoCameraCalibrationTask::setMotionData(const ImuDatas& data)
{
    d->_imuData = data;
}

void MonoCameraCalibrationTask::clearData()
{
    d->_imuData.clear();
    d->_numViews = 0;

    d->_mcmb->clearData();

    d->_calibrated = false;
}

MonoCameraCalibrationTask::Error MonoCameraCalibrationTask::startCalibration()
{
    if (d->_numViews < d->_opts.minNumViews) {
        LOG(WARNING) << "Not enough views to start calibration: View count "
                     << d->_numViews
                     << ", Min view count: " << d->_opts.minNumViews;
        return Error::NotEnoughViews;
    }

    /// Multi-Camera-Multi-Board calibration
    LOG(INFO) << "Start camera calibration.";

    d->_mcmb->startCalibration();

    const auto [visStart, visEnd] = d->_mcmb->samplePeriod();

    LOG(INFO) << "Visual timeline start " << visStart << ", end " << visEnd;

    Timeline accStamps;
    std::vector<Vector3d> accs;
    for (const auto& acc : d->_imuData.acc.d()) {
        const auto& stamp = acc.timestamp();
        if (stamp > visEnd || stamp < visStart) {
            continue;
        }

        accStamps.push_back(stamp);
        accs.push_back(acc.data());
    }

    Timeline gyroStamps;
    std::vector<Vector3d> gyros;
    for (const auto& gyro : d->_imuData.gyro.d()) {
        const auto stamp = gyro.timestamp();
        if (stamp > visEnd || stamp < visStart) {
            continue;
        }

        gyroStamps.push_back(stamp);
        gyros.push_back(gyro.data());
    }

    InertialBasedScaleEstimation ibse;
    if (!ibse.setInertialData(accStamps, accs, gyroStamps, gyros)) {
        LOG(ERROR) << "Invalid data to estimate camera imu rotation.";
        return FailedCameraImuCalibration;
    }

    Timeline visStamps;
    std::vector<Vector3d> visTranslations;
    std::vector<Quaterniond> visRotations;
    d->_mcmb->getCameraPoses(kCameraLeftId, visStamps, visTranslations,
                             visRotations);
    if (!ibse.setVisualData(visStamps, visTranslations, visRotations)) {
        LOG(ERROR) << "Invalid  camera data to estimate camera imu rotation.";
        return FailedCameraImuCalibration;
    }

    if (ibse.initialAlignmentEstimation() !=
        InertialBasedScaleEstimation::ErrorCode::Success) {
        LOG(ERROR) << "Failed to estimate left camera imu rotation.";
        return FailedCameraImuCalibration;
    }

    const auto R_c0i = ibse.imuToCameraRotation();
    d->_td_c0i = ibse.imuToCameraTimeOffset() * 1e3; // to ms

    double r, p, y;
    math::RotationMatrixToRPY(R_c0i, r, p, y);
    LOG(INFO) << "Camera to IMU rotation (roll, pitch, yaw): " << r << ", " << p
              << ", " << y << "\n"
              << R_c0i.transpose();
    LOG(INFO) << "Time offset: " << d->_td_c0i;

    const Matrix3d R_ic0 = R_c0i.transpose();
    const Vector3d t_ic0{d->_product.t_ic0.data()};
    d->_T_ic0.block<3, 3>(0, 0) = R_ic0;
    d->_T_ic0.block<3, 1>(0, 3) = t_ic0;

    // Update Meta
    const auto [minTemp, maxTemp] =
        std::ranges::minmax(d->_imuData.temperatures);
    d->_info.minTemp = minTemp;
    d->_info.maxTemp = maxTemp;
    d->_info.datetime = std::format("{:%F %X}", time::currentDatetime());

    d->_calibrated = true;

    return None;
}

bool MonoCameraCalibrationTask::calibrated() const { return d->_calibrated; }

ReportSummary MonoCameraCalibrationTask::resultAsSummary(
    const Reference& ref) const
{
    using namespace Qt::Literals::StringLiterals;

    ReportSummary::Items items;

    const auto cam0 = d->_mcmb->camera(kCameraLeftId);
    const auto f0 = cam0->_intrinsics[0];
    const cv::Point2d c0{cam0->_intrinsics[2], cam0->_intrinsics[3]};

    {
        // TODO:
        const auto rpe = 0.2;

        ReportItem item;
        item.name = tr("Rpe");
        item.value = QString::number(rpe);
        item.expectation = u"< %1"_s.arg(ref.maxRpe);
        item.toolTip = tr("Reprojection error (RPE) should be small.");
        item.pass = rpe < ref.maxRpe;
        item.used = false;

        items.push_back(item);
    }
    {
        ReportItem item;
        item.name = tr("Focal length");
        item.value = QString::number(f0);
        item.expectation =
            u"%1 +- %2"_s.arg(QString::number(d->_product.focalLength),
                              QString::number(ref.focalLengthTolerance));
        item.toolTip = tr("Camera focal length.");
        item.pass =
            std::abs(f0 - d->_product.focalLength) < ref.focalLengthTolerance;
        item.used = true;

        items.push_back(item);
    }

    {
        ReportItem item;
        item.name = tr("Principal point");
        item.value =
            u"(%1, %2)"_s.arg(QString::number(c0.x), QString::number(c0.y));
        item.expectation = u"(%1, %2) +- (%3, %4)"_s.arg(
            QString::number(d->_product.principalPoint[0]),
            QString::number(d->_product.principalPoint[1]),
            QString::number(ref.principalPointTolerance[0]),
            QString::number(ref.principalPointTolerance[1]));
        item.toolTip = tr("Camera principal point.");
        item.pass = (c0.x - d->_product.principalPoint[0]) <
                        ref.principalPointTolerance[0] &&
                    (c0.y - d->_product.principalPoint[1]) <
                        ref.principalPointTolerance[1];
        item.used = true;

        items.push_back(item);
    }

    const Matrix3d R_ic0 = d->_T_ic0.block<3, 3>(0, 0);
    // const Vector3d t_ic0 = d->_T_ic0.block<3, 1>(0, 3);

    {
        constexpr auto kImuCameraRotation = 90.;

        const auto angle = math::radToDeg(AngleAxisd{R_ic0}.angle());

        // Readibility
        double roll{0.}, pitch{0.}, yaw{0.};
        math::RotationMatrixToRPY(R_ic0, roll, pitch, yaw);

        ReportItem item;
        item.name = tr("IMU-Camera rotation");
        item.value = QString::number(angle);
        item.expectation =
            u"%1 +- %2"_s.arg(QString::number(kImuCameraRotation),
                              QString::number(ref.imuCameraRotationTolerance));
        item.toolTip =
            tr("Rotation between IMU and camera: r: %1, p: %2, y: %3")
                .arg(QString::number(math::radToDeg(roll)),
                     QString::number(math::radToDeg(pitch)),
                     QString::number(math::radToDeg(yaw)));
        item.pass = std::abs(angle - kImuCameraRotation) <
                    ref.imuCameraRotationTolerance;
        item.used = false;

        items.push_back(item);
    }
    {
        ReportItem item;
        item.name = tr("IMU-Camera time offset");
        item.value = QString::number(d->_td_c0i);
        item.expectation =
            u"abs < %1"_s.arg(QString::number(ref.maxImuCameraTimeOffset));
        item.toolTip =
            tr("Time offset between IMU and camera should be small.");
        item.pass = std::abs(d->_td_c0i) < ref.maxImuCameraTimeOffset;
        item.used = false;

        items.push_back(item);
    }

    return ReportSummary{items};
}

std::string MonoCameraCalibrationTask::resultAsYaml() const
{
    const auto cam0 = d->_mcmb->camera(kCameraLeftId);

    if (!cam0) {
        return {};
    }

    YAML::Emitter emitter;

    // clang-format off
    emitter << YAML::BeginMap
            << YAML::Key << key::kVersion << YAML::Value << YAML::DoubleQuoted << APP_VERSION_STRING
            << YAML::Key << key::kDevice << YAML::Value << d->_device
            << YAML::Key << key::kTask << YAML::Value  << d->_info
            << YAML::Key << key::kCameras << YAML::Value
                         << YAML::BeginMap
                         << YAML::Key << key::kCameraLeft << YAML::Value << YAML::convert<mcmb::Camera>::encode(*cam0)
                         << YAML::EndMap
            << YAML::Key << key::kTd << YAML::Value << d->_td_c0i
            << YAML::Key << key::kCameraToImuTransform << YAML::Value  << io::toCVYamlNode(d->_T_ic0)
            << YAML::EndMap;
    // clang-format on

    std::ostringstream oss;
    io::cvYamlHeader(oss);
    oss << emitter.c_str() << "\n";

    return oss.str();
}

} // namespace tl

namespace nlohmann {

using namespace tl;

void to_json(json& j, const tl::MonoCameraCalibrationTask::Options& opts)
{
    j[key::kMaxNumViews] = opts.maxNumViews;
    j[key::kMinNumViews] = opts.minNumViews;
    j[key::kSkipStep] = opts.skipStep;
    j[key::kMCMB] = opts.mcmbOptions;
}

void from_json(const json& j, tl::MonoCameraCalibrationTask::Options& opts)
{
    j.at(key::kMaxNumViews).get_to(opts.maxNumViews);
    j.at(key::kMinNumViews).get_to(opts.minNumViews);
    j.at(key::kSkipStep).get_to(opts.skipStep);
    j.at(key::kMCMB).get_to(opts.mcmbOptions);
}

void to_json(json& j, const tl::MonoCameraCalibrationTask::Reference& ref)
{
    j[key::kMaxRpe] = ref.maxRpe;
    j[key::kFocalLengthTolerance] = ref.focalLengthTolerance;
    j[key::kPrincipalPointTolerance] = ref.principalPointTolerance;
    j[key::kImuCameraRotationTolerance] = ref.imuCameraRotationTolerance;
    j[key::kMaxImuCameraTimeOffset] = ref.maxImuCameraTimeOffset;
}

void from_json(const json& j, tl::MonoCameraCalibrationTask::Reference& ref)
{
    j.at(key::kMaxRpe).get_to(ref.maxRpe);
    j.at(key::kFocalLengthTolerance).get_to(ref.focalLengthTolerance);
    j.at(key::kPrincipalPointTolerance).get_to(ref.principalPointTolerance);
    j.at(key::kImuCameraRotationTolerance)
        .get_to(ref.imuCameraRotationTolerance);
    j.at(key::kMaxImuCameraTimeOffset).get_to(ref.maxImuCameraTimeOffset);
}

} // namespace nlohmann
