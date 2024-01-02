#pragma once

#include <string>

#include <json/json.hpp>
#include <yaml-cpp/yaml.h>

#include <AxImu/ImuIntrinsics>
#include <AxImu/ImuTypes>

// For nlohmann::json
namespace thoht {

class ImuNoise;

void to_json(nlohmann::json& json, const ImuNoise& noise);
void from_json(const nlohmann::json& json, ImuNoise& noise);

template <typename T>
void to_json(nlohmann::json& json, const ImuIntrinsics_<T>& intrinsics)
{
    // TODO:
}

template <typename T>
void from_json(const nlohmann::json& json, ImuIntrinsics_<T>& intrinsics)
{
    // TODO:
}

} // namespace thoht

namespace thoht::io {

YAML::Node toYamlNode(const ImuIntrinsics& intrinsics, imu::Type type);
YAML::Node toYamlNode(const ImuNoise& noise, imu::Type type);

} // namespace thoht::io

// For yaml-cpp
namespace YAML {

template <>
struct convert<thoht::ImuNoise>
{
    static Node encode(const thoht::ImuNoise& noise);
    static bool decode(const Node& node, thoht::ImuNoise& noise);
};

template <typename T>
struct convert<thoht::ImuIntrinsics_<T>>
{
    using _ImuIntrinsics = thoht::ImuIntrinsics_<T>;

    static Node encode(const _ImuIntrinsics& intrinsics)
    {
        // TODO:
        return {};
    }

    static bool decode(const Node& node, _ImuIntrinsics& intrinsics)
    {
        // TODO:
        return false;
    }
};

} // namespace YAML
