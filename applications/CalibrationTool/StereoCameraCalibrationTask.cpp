#include "StereoCameraCalibrationTask.h"

#include <glog/logging.h>
#include <opencv2/core.hpp>

#include <tCalib/CalibrationData>
#include <tCalib/InertialBasedScaleEstimation>
#include <tCalib/SplineTrajectoryEstimator2>
#include <tCalib/IO/CalibrationIO>
#include <tCalib/IO/CameraIO>
#include <tCalib/IO/EigenIO>
#include <tCalib/MCMB/Camera>
#include <tCamera/Types>
#include <tCore/Math>
#include <tCore/TimeUtils>
#include <tMath/Eigen/Utils>
#include <tMotion/ImuData>

#include <csv-parser/csv.hpp>

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

namespace {

} // namespace

namespace key {
constexpr char kMaxNumViews[]{"max_num_views"};
constexpr char kMinNumViews[]{"min_num_views"};
constexpr char kSkipStep[]{"skip_step"};
constexpr char kRefineCorner[]{"refine_corners"};
constexpr char kMCMB[]{"mcmb"};

constexpr char kVersion[]{"version"};
constexpr char kDevice[]{"device"};
constexpr char kTask[]{"task"};

// FIXME: Duplicated code
constexpr char kCameras[]{"cameras"};
constexpr char kCameraLeft[]{"cam0"};
constexpr char kCameraRight[]{"cam1"};
constexpr char kFocalLength[]{"focal_length"};
constexpr char kTd[]{"td"};
constexpr char kLeftCamToImu[]{"body_T_cam0"};
constexpr char kRightCamToImu[]{"body_T_cam1"};

constexpr char kMaxRpe[]{"max_rpe"};
constexpr char kFocalLengthTolerance[]{"focal_length_tolerance"};
constexpr char kPrincipalPointTolerance[]{"principal_point_tolerance"};
constexpr char kMaxPrincipalPointDiff[]{"max_principal_point_diff"};
constexpr char kBaselineTolerance[]{"baseline_tolerance"};
constexpr char kInterCameraRotationTolerance[]{
    "inter_camera_rotation_tolerance"};
constexpr char kImuCameraRotationTolerance[]{"imu_camera_rotation_tolerance"};
constexpr char kMaxImuCameraTimeOffset[]{"max_imu_camera_time_offset"};
} // namespace key

///------- StereoCameraCalibrationTask::Impl starts from here
class StereoCameraCalibrationTask::Impl
{
public:
    Impl();

    void updateOptions(const Options& options);

public:
    Options _opts;
    DeviceInfo _device;
    ProductInfo _product;
    TaskInfo _info;

    std::unique_ptr<AprilTag> _target;
    std::unique_ptr<mcmb::Calibration> _mcmb;

    // Result
    Eigen::Matrix4d _T_c0c1{Eigen::Matrix4d::Identity()};
    Eigen::Matrix4d _T_ic0{Eigen::Matrix4d::Identity()};
    Eigen::Matrix4d _T_ic1{Eigen::Matrix4d::Identity()};
    double _td_c0i{0.}, _td_c1i{0.};

    // Cached data
    ImuDatas _imuData;
    int _numViews{0};
    // FIXME: Stupid, dont need this
    bool _calibrated{false};
};

StereoCameraCalibrationTask::Impl::Impl()
    : _mcmb(std::make_unique<mcmb::Calibration>())
{
}

void StereoCameraCalibrationTask::Impl::updateOptions(const Options& opts)
{
    _opts = opts;
    _mcmb->setOptions(opts.mcmbOptions);
}

///------- StereoCameraCalibrationTask starts from here
StereoCameraCalibrationTask::StereoCameraCalibrationTask(const Options& opts)
    : d(std::make_unique<Impl>())
{
    setOptions(opts);
}

StereoCameraCalibrationTask::~StereoCameraCalibrationTask() = default;

const TaskInfo& StereoCameraCalibrationTask::info() const { return d->_info; }

void StereoCameraCalibrationTask::resetTaskInfo() { d->_info = TaskInfo{}; }

const DeviceInfo& StereoCameraCalibrationTask::deviceInfo() const
{
    return d->_device;
}

void StereoCameraCalibrationTask::setDeviceInfo(const DeviceInfo& device)
{
    d->_device = device;
}

const StereoCameraCalibrationTask::Options&
StereoCameraCalibrationTask::options() const
{
    return d->_opts;
}

void StereoCameraCalibrationTask::setOptions(const Options& opts)
{
    d->updateOptions(opts);
}

bool StereoCameraCalibrationTask::setFromJson(const std::string& json)
{
    try {
        const auto j = nlohmann::json::parse(json);

        Options opts;
        j.get_to(opts);
        setOptions(opts);
    }
    catch (const nlohmann::detail::parse_error& e) {
        LOG(WARNING) << "Failed to parse stereo camera calibration task json: "
                     << e.what();
        return false;
    }
    catch (const nlohmann::detail::out_of_range& e) {
        LOG(WARNING)
            << "Incompatiable stereo camera calibration task options json: "
            << e.what();
        return false;
    }

    return true;
}

bool StereoCameraCalibrationTask::setTargetFromJson(const std::string& json)
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

void StereoCameraCalibrationTask::setProductInfo(const ProductInfo& info)
{
    d->_product = info;
}

void StereoCameraCalibrationTask::addStereoData(const StereoImages& stereo,
                                                const StereoImageInfo& info)
{
    if (!d->_target) {
        return;
    }

    if (!stereo.isValid()) {
        return;
    }

    if (d->_numViews >= d->_opts.maxNumViews) {
        return;
    }

    auto isValidDetections = [](const AprilTag::Board::Detections& detections) {
        return std::ranges::any_of(detections, [](const auto& detection) {
            return !detection.empty();
        });
    };

    const auto& boardOptions = d->_opts.mcmbOptions.boardOptions;
    const auto& timestamp = stereo.timestamp;
    const auto detections0 =
        d->_target->detectBoards(stereo.left, boardOptions);
    const auto detections1 =
        d->_target->detectBoards(stereo.right, boardOptions);

    if (!isValidDetections(detections0) && !isValidDetections(detections1)) {
        LOG(WARNING) << "Failed to detect tags in both views.";
        return;
    }

    // We can't tell if a board detection result is valid or not without
    // iterating them one by one, so leave it to Calibration class.
    if (d->_opts.refineCorners) {
        // ...
    }

    if ((d->_numViews % d->_opts.skipStep) == 0) {
        d->_mcmb->addBoardDetections(d->_numViews, kCameraLeftId, detections0,
                                     timestamp, info.left);
        d->_mcmb->addBoardDetections(d->_numViews, kCameraRightId, detections1,
                                     timestamp, info.right);
    }

    ++d->_numViews;
}

void StereoCameraCalibrationTask::addMotionData(const ImuData& data)
{
    d->_imuData.push_back(data);
}

void StereoCameraCalibrationTask::setMotionData(const ImuDatas& data)
{
    d->_imuData = data;
}

void StereoCameraCalibrationTask::clearData()
{
    // Cached data
    d->_imuData.clear();
    d->_numViews = 0;

    // Result
    d->_T_c0c1.setIdentity();
    d->_T_ic0.setIdentity();
    d->_T_ic1.setIdentity();
    d->_td_c0i = 0.;
    d->_td_c1i = 0.;

    d->_mcmb->clearData();

    d->_calibrated = false;
}

StereoCameraCalibrationTask::Error
StereoCameraCalibrationTask::startCalibration()
{
    if (d->_numViews < d->_opts.minNumViews) {
        LOG(WARNING) << "Not enough views to start calibration: "
                        "View count "
                     << d->_numViews
                     << ", Min view count: " << d->_opts.minNumViews;
        return NotEnoughViews;
    }

    /// Multi-Camera-Multi-Board calibration
    LOG(INFO) << "Start camera calibration.";
    d->_mcmb->startCalibration();

    // Camera0 is Identity
    d->_mcmb->getExtrinsics(d->_T_c0c1);

    LOG(INFO) << "Extrinsics: \n" << d->_T_c0c1;

    /// Camera-IMU rotation estimation
    const auto [visStart, visEnd] = d->_mcmb->samplePeriod();

    LOG(INFO) << "Visual timeline start " << visStart << ", end " << visEnd;

    Timeline accStamps;
    std::vector<Vector3d> accs;
    {
        std::ofstream fout{"acc.csv"};
        auto writer = csv::make_csv_writer(fout);
        writer << std::array{"stamp", "acc_x", "acc_y", "acc_z"};
        for (const auto& acc : d->_imuData.acc.d()) {
            const auto& stamp = acc.timestamp();
            if (stamp > visEnd || stamp < visStart) {
                continue;
            }

            accStamps.push_back(stamp);
            accs.push_back(acc.data());

            writer << std::array{stamp, acc.x(), acc.y(), acc.z()};
        }
    }

    Timeline gyroStamps;
    std::vector<Vector3d> gyros;
    {
        std::ofstream fout{"gyr.csv"};
        auto writer = csv::make_csv_writer(fout);
        writer << std::array{"stamp", "gyr_x", "gyr_y", "gyr_z"};
        for (const auto& gyro : d->_imuData.gyro.d()) {
            const auto stamp = gyro.timestamp();
            if (stamp > visEnd || stamp < visStart) {
                continue;
            }

            gyroStamps.push_back(stamp);
            gyros.push_back(gyro.data());

            writer << std::array{stamp, gyro.x(), gyro.y(), gyro.z()};
        }
    }

    InertialBasedScaleEstimation ibse;
    if (!ibse.setInertialData(accStamps, accs, gyroStamps, gyros)) {
        LOG(ERROR) << "Invalid data to estimate camera imu rotation.";
        return FailedCameraImuCalibration;
    }

    auto estimateScale = [this, &ibse](int camId, Eigen::Matrix3d& R_ci,
                                       double& td_ci) -> bool {
        Timeline visStamps;
        std::vector<Vector3d> visTranslations;
        std::vector<Quaterniond> visRotations;
        d->_mcmb->getCameraPoses(camId, visStamps, visTranslations,
                                 visRotations);
        if (!ibse.setVisualData(visStamps, visTranslations, visRotations)) {
            LOG(ERROR) << "Invalid camera " << camId
                       << " data to estimate camera imu rotation.";
            return false;
        }

        if (ibse.initialAlignmentEstimation() !=
            InertialBasedScaleEstimation::ErrorCode::Success) {
            LOG(ERROR) << "Failed to estimate left camera imu rotation.";
            return false;
        }

        {
            std::ofstream fout{std::format("cam{}.csv", camId)};
            auto writer = csv::make_csv_writer(fout);
            writer << std::array{"stamp", "t_x", "t_y", "t_z",
                                 "q_w",   "q_x", "q_y", "q_z"};
            for (size_t i{0}; i < visStamps.size(); ++i) {
                const auto& t = visStamps[i];
                const auto& tvec = visTranslations[i];
                const auto& quat = visRotations[i];

                writer << std::array{t,        tvec.x(), tvec.y(), tvec.z(),
                                     quat.w(), quat.x(), quat.y(), quat.z()};
            }
        }

        R_ci = ibse.imuToCameraRotation();
        td_ci = ibse.imuToCameraTimeOffset() * 1e3;

        double r, p, y;
        math::RotationMatrixToRPY(R_ci, r, p, y);
        LOG(INFO) << "Camera " << camId
                  << " to IMU rotation (roll, pitch, yaw): " << r << ", " << p
                  << ", " << y << "\n"
                  << R_ci.transpose();
        LOG(INFO) << "Time offset: " << td_ci;
        return true;
    };

    Matrix3d R_c0i, R_c1i;
    [[maybe_unused]] const auto ret0 =
        estimateScale(kCameraLeftId, R_c0i, d->_td_c0i);
    [[maybe_unused]] const auto ret1 =
        estimateScale(kCameraRightId, R_c1i, d->_td_c1i);

    const Matrix3d R_ic0 = R_c0i.transpose();
    const Vector3d t_ic0{d->_product.t_ic0.data()};
    d->_T_ic0.block<3, 3>(0, 0) = R_ic0;
    d->_T_ic0.block<3, 1>(0, 3) = t_ic0;

    const Matrix3d R_c0c1 = d->_T_c0c1.block<3, 3>(0, 0);
    const Vector3d t_c0c1 = d->_T_c0c1.block<3, 1>(0, 3);

    const Matrix3d R_ic1 = R_ic0 * R_c0c1.transpose();
    const Vector3d t_ic1 = t_ic0 - R_ic1 * t_c0c1;
    d->_T_ic1.block<3, 3>(0, 0) = R_ic1;
    d->_T_ic1.block<3, 1>(0, 3) = t_ic1;

    SplineTrajectoryEstimator<6> spline;

    // Update Meta
    const auto [minTemp, maxTemp] =
        std::ranges::minmax(d->_imuData.temperatures);
    d->_info.minTemp = minTemp;
    d->_info.maxTemp = maxTemp;
    d->_info.datetime = std::format("{:%F %X}", time::currentDatetime());

    d->_calibrated = true;

    return None;
}

bool StereoCameraCalibrationTask::calibrated() const { return d->_calibrated; }

ReportSummary StereoCameraCalibrationTask::resultAsSummary(
    const Reference& ref) const
{
    using namespace Qt::Literals::StringLiterals;

    ReportSummary::Items items;

    const auto cam0 = d->_mcmb->camera(kCameraLeftId);
    const auto cam1 = d->_mcmb->camera(kCameraRightId);

    const auto f0 = cam0->_intrinsics[0];
    const auto f1 = cam1->_intrinsics[0];
    const cv::Point2d c0{cam0->_intrinsics[2], cam0->_intrinsics[3]};
    const cv::Point2d c1{cam1->_intrinsics[2], cam1->_intrinsics[3]};
    {
        // TODO:
        const auto rpe0 = 0.21;
        const auto rpe1 = 0.22;

        ReportItem item;
        item.name = tr("Rpe");
        item.value =
            u"[%1, %2]"_s.arg(QString::number(rpe0), QString::number(rpe1));
        item.expectation = u"< %1"_s.arg(ref.maxRpe);
        item.toolTip = tr("Reprojection error (RPE) should be small.");
        item.pass = std::ranges::all_of(
            std::array{rpe0, rpe1},
            [max = ref.maxRpe](const auto& rpe) { return rpe < max; });
        item.used = false;

        items.push_back(item);
    }
    {
        ReportItem item;
        item.name = tr("Focal length");
        item.value =
            u"[%1, %2]"_s.arg(QString::number(f0), QString::number(f1));
        item.expectation =
            u"%1 +- %2"_s.arg(QString::number(d->_product.focalLength),
                              QString::number(ref.focalLengthTolerance));
        item.toolTip = tr("Camera [left, right] focal length.");
        item.pass = std::ranges::all_of(
            std::array{f0, f1},
            [exp = d->_product.focalLength, tol = ref.focalLengthTolerance](
                const auto& f) { return std::abs(f - exp) < tol; });
        item.used = true;

        items.push_back(item);
    }

    {
        ReportItem item;
        item.name = tr("Principal point");
        item.value = u"(%1, %2)\n(%3, %4)"_s.arg(
            QString::number(c0.x), QString::number(c0.y), QString::number(c1.x),
            QString::number(c1.y));
        item.expectation = u"(%1, %2) +- (%3, %4)"_s.arg(
            QString::number(d->_product.principalPoint[0]),
            QString::number(d->_product.principalPoint[1]),
            QString::number(ref.principalPointTolerance[0]),
            QString::number(ref.principalPointTolerance[1]));
        item.toolTip = tr("Camera [left, right] principal point.");
        item.pass =
            std::ranges::all_of(std::array{c0, c1}, [&, this](const auto& c) {
                return (c.x - d->_product.principalPoint[0]) <
                           ref.principalPointTolerance[0] &&
                       (c.y - d->_product.principalPoint[1]) <
                           ref.principalPointTolerance[1];
            });
        item.used = true;

        items.push_back(item);
    }
    {
        const auto diff = c0 - c1;

        ReportItem item;
        item.name = tr("Principal point consistency");
        item.value =
            u"(%1, %2)"_s.arg(QString::number(diff.x), QString::number(diff.y));
        item.expectation = u"abs < (%1, %2)"_s.arg(
            QString::number(ref.maxPrincipalPointDiff[0]),
            QString::number(ref.maxPrincipalPointDiff[1]));
        item.toolTip = tr(
            "Principal point diff between left and right camera should small.");
        item.pass = diff.x < ref.maxPrincipalPointDiff[0] &&
                    diff.y < ref.maxPrincipalPointDiff[1];
        item.used = true;

        items.push_back(item);
    }

    const Matrix3d R_c0c1 = d->_T_c0c1.block<3, 3>(0, 0);
    const Vector3d t_c0c1 = d->_T_c0c1.block<3, 1>(0, 3);

    {
        const auto baseline = t_c0c1.norm();
        ReportItem item;
        item.name = tr("Baseline");
        item.value = QString::number(baseline);
        item.expectation =
            u"%1 +- %2"_s.arg(QString::number(d->_product.baseline),
                              QString::number(ref.baselineTolerance));
        item.toolTip =
            tr("Stereo camera baseline should be identical with design value.");
        item.pass =
            std::abs(baseline - d->_product.baseline) < ref.baselineTolerance;
        item.used = true;
        items.push_back(item);
    }
    {
        const auto angle = math::radToDeg(AngleAxisd{R_c0c1}.angle());

        // Readibility
        double roll{0.}, pitch{0.}, yaw{0.};
        math::RotationMatrixToRPY(R_c0c1, roll, pitch, yaw);

        ReportItem item;
        item.name = tr("Inter-Camera rotation");
        item.value = QString::number(angle);
        item.expectation = u"%1 +- %2"_s.arg(
            QString::number(0.),
            QString::number(ref.interCameraRotationTolerance));
        item.toolTip = u"(r: %1, p: %2, y: %3)"_s.arg(
            QString::number(math::radToDeg(roll)),
            QString::number(math::radToDeg(pitch)),
            QString::number(math::radToDeg(yaw)));
        item.pass = std::abs(angle) < ref.interCameraRotationTolerance;
        item.used = true;

        items.push_back(item);
    }

    const Matrix3d R_ic0 = d->_T_ic0.block<3, 3>(0, 0);
    // const Vector3d t_ic0 = d->_T_ic0.block<3, 1>(0, 3);
    const Matrix3d R_ic1 = d->_T_ic1.block<3, 3>(0, 0);
    // const Vector3d t_ic1 = d->_T_ic1.block<3, 1>(0, 3);

    {
        constexpr auto kImuCameraRotation = 90.;

        const auto angle0 = math::radToDeg(AngleAxisd{R_ic0}.angle());
        const auto angle1 = math::radToDeg(AngleAxisd{R_ic1}.angle());

        ReportItem item;
        item.name = tr("IMU-Camera rotation");
        item.value =
            u"[%1, %2]"_s.arg(QString::number(angle0), QString::number(angle1));
        item.expectation =
            u"%1 +- %2"_s.arg(QString::number(kImuCameraRotation),
                              QString::number(ref.imuCameraRotationTolerance));
        item.toolTip = tr("Rotation between IMU and camera [left, right].");
        item.pass = std::ranges::all_of(
            std::array{angle0, angle1},
            [exp = kImuCameraRotation, tol = ref.imuCameraRotationTolerance](
                const auto& angle) { return std::abs(angle - exp) < tol; });
        item.used = true;

        items.push_back(item);
    }
    {
        ReportItem item;
        item.name = tr("IMU-Camera time offset");
        item.value = u"[%1, %2]"_s.arg(QString::number(d->_td_c0i),
                                       QString::number(d->_td_c1i));
        item.expectation =
            u"abs < %1"_s.arg(QString::number(ref.maxImuCameraTimeOffset));
        item.toolTip = tr("Time offset between IMU and camera [left, right] "
                          "should be small.");
        item.pass = std::ranges::all_of(
            std::array{d->_td_c0i, d->_td_c1i},
            [max = ref.maxImuCameraTimeOffset](const auto& td) {
                return std::abs(td) < max;
            });
        item.used = false;

        items.push_back(item);
    }

    return ReportSummary{items};
}

// FIXME: This part is duplicated in io_calib
std::string StereoCameraCalibrationTask::resultAsYaml() const
{
    // Cameras
    const auto cam0 = d->_mcmb->camera(kCameraLeftId);
    const auto cam1 = d->_mcmb->camera(kCameraRightId);

    if (!cam0 || !cam1) {
        return {};
    }

    const auto focal = (cam0->_intrinsics[0] + cam1->_intrinsics[0]) * 0.5;

    YAML::Emitter emitter;

    // clang-format off
    emitter << YAML::BeginMap
            << YAML::Key << key::kVersion << YAML::Value << YAML::DoubleQuoted << APP_VERSION_STRING
            << YAML::Key << key::kDevice << YAML::Value << d->_device
            << YAML::Key << key::kTask << YAML::Value  << d->_info
            << YAML::Key << key::kCameras << YAML::Value
                         << YAML::BeginMap
                         << YAML::Key << key::kCameraLeft << YAML::Value << YAML::convert<mcmb::Camera>::encode(*cam0)
                         << YAML::Key << key::kCameraRight << YAML::Value << YAML::convert<mcmb::Camera>::encode(*cam1)
                         << YAML::EndMap
            << YAML::Key << key::kFocalLength << YAML::Value  << focal
            << YAML::Key << key::kTd << YAML::Value << YAML::Flow
                         << YAML::BeginSeq << d->_td_c0i << d->_td_c0i << YAML::EndSeq
            << YAML::Key << key::kLeftCamToImu << YAML::Value  << io::toCVYamlNode(d->_T_ic0)
            << YAML::Key << key::kRightCamToImu << YAML::Value  << io::toCVYamlNode(d->_T_ic1)
            << YAML::EndMap;
    // clang-format on

    std::ostringstream oss;
    io::cvYamlHeader(oss);
    oss << emitter.c_str() << "\n";

    return oss.str();
}

bool StereoCameraCalibrationTask::saveDetailResultTo(
    const std::string& root) const
{
    d->_mcmb->saveResults(root);

    return true;
}

} // namespace tl

namespace nlohmann {

using namespace tl;

void to_json(json& j, const tl::StereoCameraCalibrationTask::Options& opts)
{
    j[key::kMaxNumViews] = opts.maxNumViews;
    j[key::kMinNumViews] = opts.minNumViews;
    j[key::kSkipStep] = opts.skipStep;
    j[key::kRefineCorner] = opts.refineCorners;
    j[key::kMCMB] = opts.mcmbOptions;
}

void from_json(const json& j, tl::StereoCameraCalibrationTask::Options& opts)
{
    j.at(key::kMaxNumViews).get_to(opts.maxNumViews);
    j.at(key::kMinNumViews).get_to(opts.minNumViews);
    j.at(key::kSkipStep).get_to(opts.skipStep);
    j.at(key::kRefineCorner).get_to(opts.refineCorners);
    j.at(key::kMCMB).get_to(opts.mcmbOptions);
}

void to_json(json& j, const tl::StereoCameraCalibrationTask::Reference& ref)
{
    j[key::kMaxRpe] = ref.maxRpe;
    j[key::kFocalLengthTolerance] = ref.focalLengthTolerance;
    j[key::kPrincipalPointTolerance] = ref.principalPointTolerance;
    j[key::kMaxPrincipalPointDiff] = ref.maxPrincipalPointDiff;
    j[key::kBaselineTolerance] = ref.baselineTolerance;
    j[key::kInterCameraRotationTolerance] = ref.interCameraRotationTolerance;
    j[key::kImuCameraRotationTolerance] = ref.imuCameraRotationTolerance;
    j[key::kMaxImuCameraTimeOffset] = ref.maxImuCameraTimeOffset;
}

void from_json(const json& j, tl::StereoCameraCalibrationTask::Reference& ref)
{
    j.at(key::kMaxRpe).get_to(ref.maxRpe);
    j.at(key::kFocalLengthTolerance).get_to(ref.focalLengthTolerance);
    j.at(key::kPrincipalPointTolerance).get_to(ref.principalPointTolerance);
    j.at(key::kMaxPrincipalPointDiff).get_to(ref.maxPrincipalPointDiff);
    j.at(key::kBaselineTolerance).get_to(ref.baselineTolerance);
    j.at(key::kInterCameraRotationTolerance)
        .get_to(ref.interCameraRotationTolerance);
    j.at(key::kImuCameraRotationTolerance)
        .get_to(ref.imuCameraRotationTolerance);
    j.at(key::kMaxImuCameraTimeOffset).get_to(ref.maxImuCameraTimeOffset);
}

} // namespace nlohmann
