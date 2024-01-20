#include "io_calib.h"

#include <format>

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <tCalibration/CameraIO>
#include <tCalibration/EigenIO>
#include <tCalibration/ImuIO>
#include <tCamera/Camera>
#include <tMotion/ImuNoise>

namespace tl::io {

namespace key {
constexpr char kCalibDateTime[]{"calibration_date"};
constexpr char kUuid[]{"uuid"};
constexpr char kHardwareAddr[]{"hardware_addr"};
constexpr char kDeviceSN[]{"device_sn"};
constexpr char kCameras[]{"cameras"};
constexpr char kCameraLeft[]{"cam0"};
constexpr char kCameraRight[]{"cam1"};
constexpr char kFocalLength[]{"focal_length"};
constexpr char kLeftCamToImu[]{"body_T_cam0"};
constexpr char kRightCamToImu[]{"body_T_cam1"};
constexpr char kGnssToImu[]{"body_T_gnss"};
constexpr char kGnssToBase[]{"baselink_T_gnss"};
constexpr char kWheelToImu[]{"body_T_wheel"};
constexpr char kBaseline[]{"baseline"};
} // namespace key

std::string toVioYamlString(const Camera& camLeft, const Camera& camRight,
                            const Eigen::Matrix4d& iTcamleft,
                            const Eigen::Matrix4d& iTcamright,
                            const CalibMetaData& meta)
{
    std::ostringstream oss;
    oss << "%YAML:1.0"
           "\n"
           "---"
           "\n";

    // Meta.
    // OpenCV yaml cant handle complicated string without quote, WTF!!!
    // YAML::Node metaNode;
    // metaNode[key::kUuid] = meta.uuid;
    // metaNode[key::kCalibDateTime] = meta.calibDateTime;
    oss << key::kUuid << ": " << std::quoted(meta.uuid) << "\n";
    oss << key::kCalibDateTime << ": " << std::quoted(meta.calibDateTime)
        << "\n";
    oss << key::kHardwareAddr << ": " << std::quoted(meta.hardwareAddr) << "\n";
    oss << key::kDeviceSN << ": " << std::quoted(meta.deviceSN) << "\n";

    // Camera
    YAML::Node cameras;
    cameras[key::kCameras][key::kCameraLeft] = toCamOdoCalYamlNode(camLeft);
    cameras[key::kCameras][key::kCameraRight] = toCamOdoCalYamlNode(camRight);
    oss << cameras << "\n";

    // Magic focal length (???)
    constexpr double kMagicFocal = 328.68046;
    YAML::Node focalLength;
    focalLength[key::kFocalLength] = kMagicFocal;
    oss << focalLength << "\n";

    // Geometry
    YAML::Node transforms;
    transforms[key::kLeftCamToImu] = toCVYamlNode(iTcamleft);
    transforms[key::kRightCamToImu] = toCVYamlNode(iTcamright);
    oss << transforms << std::endl;

    return oss.str();
}

namespace internal {

bool fromVioYamlNode(const YAML::Node& root, Camera& left, Camera& right,
                     double& focalLength, Eigen::Matrix4d& leftCameraToImu,
                     Eigen::Matrix4d& rightCameraToImu)
{
    const auto camerasNode = root[key::kCameras];
    const auto leftCameraToImuNode = root[key::kLeftCamToImu];
    const auto rightCameraToImuNode = root[key::kRightCamToImu];
    const auto focalLengthNode = root[key::kFocalLength];

    if (!camerasNode || !leftCameraToImuNode || !rightCameraToImuNode ||
        !focalLengthNode) {
        return false;
    }

    // Cameras
    const bool readCameras =
        fromCamOdoCalYamlNode(camerasNode[key::kCameraLeft], left) &&
        fromCamOdoCalYamlNode(camerasNode[key::kCameraRight], right);

    // Geometry
    const bool readGeometry =
        fromCVYamlNode(leftCameraToImuNode, leftCameraToImu) &&
        fromCVYamlNode(rightCameraToImuNode, rightCameraToImu);

    focalLength = focalLengthNode.as<double>();

    return readCameras && readGeometry;
}

} // namespace internal

bool fromVioYamlString(const std::string& bytes, Camera& left, Camera& right,
                       double& focalLength, Eigen::Matrix4d& leftCameraToImu,
                       Eigen::Matrix4d& rightCameraToImu)
{
    return internal::fromVioYamlNode(YAML::Load(bytes), left, right,
                                     focalLength, leftCameraToImu,
                                     rightCameraToImu);
}

bool loadFromVioYamlFile(const std::string& filename, Camera& left,
                         Camera& right, double& focalLength,
                         Eigen::Matrix4d& leftCameraToImu,
                         Eigen::Matrix4d& rightCameraToImu)
{
    return internal::fromVioYamlNode(YAML::LoadFile(filename), left, right,
                                     focalLength, leftCameraToImu,
                                     rightCameraToImu);
}

std::string taskDirName(const std::string& uuid)
{
    return std::format("task-{0}", uuid);
}

} // namespace tl::io
