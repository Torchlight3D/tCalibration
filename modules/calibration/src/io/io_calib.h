#pragma once

#include <string>

#include <Eigen/Core>

#include <tMotion/ImuIntrinsics>

namespace tl {

class Camera;
class ImuNoise;

namespace io {

inline constexpr char kCalibParameterFilename[]{"cam_parameters.yaml"};
inline constexpr char kSaveDataDir[]{"SavedData"};

struct CalibMetaData
{
    std::string uuid{};
    std::string calibDateTime{};
    std::string hardwareAddr{};
    std::string deviceSN{};
};

std::string toVioYamlString(const Camera& left, const Camera& right,
                            const Eigen::Matrix4d& leftCameraToImu,
                            const Eigen::Matrix4d& rightCameraToImu,
                            const CalibMetaData& meta);

// TODO: Not sure how to use meta data
bool fromVioYamlString(const std::string& bytes, Camera& left, Camera& right,
                       double& focalLength, Eigen::Matrix4d& leftCameraToImu,
                       Eigen::Matrix4d& rightCameraToImu);

bool loadFromVioYamlFile(const std::string& filename, Camera& left,
                         Camera& right, double& focalLength,
                         Eigen::Matrix4d& leftCameraToImu,
                         Eigen::Matrix4d& rightCameraToImu);

std::string taskDirName(const std::string& uuid);

} // namespace io

} // namespace tl
