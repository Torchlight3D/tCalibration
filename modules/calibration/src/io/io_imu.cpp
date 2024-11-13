#include "io_imu.h"

#include <tCalibration/EigenIO>
#include <tMotion/ImuNoise>

namespace tl::io {

namespace key {

constexpr char kImu[]{"imu"};
constexpr char kAccelerator[]{"acc"};
constexpr char kGyroscope[]{"gyr"};

// ImuIntrinsics
constexpr char kAccIntrinsics[]{"acc_intrinsics"};
constexpr char kAccScale[]{"acc_scale"};
constexpr char kAccBias[]{"acc_bias"};
constexpr char kGyrIntrinsics[]{"gyr_intrinsics"};
constexpr char kGyrScale[]{"gyr_scale"};
constexpr char kGyrBias[]{"gyr_bias"};

// ImuNoise
constexpr char kAccNoise[]{"acc_noise"};
constexpr char kAccRandomWalk[]{"acc_random_walk"};
constexpr char kGyrNoise[]{"gyr_noise"};
constexpr char kGyrRandomWalk[]{"gyr_random_walk"};

} // namespace key

YAML::Node toYamlNode(const ImuIntrinsics& imu, ImuType type)
{
    YAML::Node node;
    switch (type) {
        case ImuType::Accelerator:
            node[key::kAccIntrinsics] = toCVYamlNode(imu.toCompactMatrix());
            node[key::kAccBias] = toCVYamlNode(imu.bias());
            break;
        case ImuType::Gyroscope:
            node[key::kGyrIntrinsics] = toCVYamlNode(imu.toCompactMatrix());
            node[key::kGyrBias] = toCVYamlNode(imu.bias());
            break;
        default:
            break;
    }

    return node;
}

YAML::Node toYamlNode(const ImuNoise& imu, ImuType type)
{
    YAML::Node node;
    switch (type) {
        case ImuType::Accelerator:
            node[key::kAccNoise] = imu.noise();
            node[key::kAccRandomWalk] = imu.randomWalk();
            break;
        case ImuType::Gyroscope:
            node[key::kGyrNoise] = imu.noise();
            node[key::kGyrRandomWalk] = imu.randomWalk();
            break;
        default:
            break;
    }

    return node;
}

} // namespace tl::io

namespace tl {

void to_json(nlohmann::json& json, const ImuNoise& noise)
{
    // TODO:
}

void from_json(const nlohmann::json& json, ImuNoise& noise)
{
    // TODO:
}

} // namespace tl

namespace YAML {

using ImuNoiseConverter = convert<tl::ImuNoise>;

Node ImuNoiseConverter::encode(const tl::ImuNoise& noise)
{
    // TODO:
    return {};
}

bool ImuNoiseConverter::decode(const Node& node, tl::ImuNoise& noise)
{
    // TODO:
    return false;
}

} // namespace YAML
