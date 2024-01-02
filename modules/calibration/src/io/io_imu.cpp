#include "io_imu.h"

#include <AxCalib/EigenIO>
#include <AxImu/ImuNoise>

namespace thoht::io {

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

YAML::Node toYamlNode(const ImuIntrinsics& imu, imu::Type type)
{
    YAML::Node node;
    switch (type) {
        case imu::Type::Accelerator:
            node[key::kAccIntrinsics] = toCVYamlNode(imu.toCompactMatrix());
            node[key::kAccBias] = toCVYamlNode(imu.bias());
            break;
        case imu::Type::Gyroscope:
            node[key::kGyrIntrinsics] = toCVYamlNode(imu.toCompactMatrix());
            node[key::kGyrBias] = toCVYamlNode(imu.bias());
            break;
        default:
            break;
    }

    return node;
}

YAML::Node toYamlNode(const ImuNoise& imu, imu::Type type)
{
    YAML::Node node;
    switch (type) {
        case imu::Type::Accelerator:
            node[key::kAccNoise] = imu.noise();
            node[key::kAccRandomWalk] = imu.randomWalk();
            break;
        case imu::Type::Gyroscope:
            node[key::kGyrNoise] = imu.noise();
            node[key::kGyrRandomWalk] = imu.randomWalk();
            break;
        default:
            break;
    }

    return node;
}

} // namespace thoht::io

namespace thoht {

void to_json(nlohmann::json& json, const ImuNoise& noise)
{
    // TODO:
}

void from_json(const nlohmann::json& json, ImuNoise& noise)
{
    // TODO:
}

} // namespace thoht

namespace YAML {

using ImuNoiseConverter = convert<thoht::ImuNoise>;

Node ImuNoiseConverter::encode(const thoht::ImuNoise& noise)
{
    // TODO:
    return {};
}

bool ImuNoiseConverter::decode(const Node& node, thoht::ImuNoise& noise)
{
    // TODO:
    return false;
}

} // namespace YAML
