#pragma once

#include <json/json.hpp>
#include <yaml-cpp/yaml.h>

#include <tMotion/ImuIntrinsics>
#include <tMotion/ImuTypes>

// For nlohmann::json
namespace tl {

class ImuNoise;

void to_json(nlohmann::json& json, const ImuNoise& noise);
void from_json(const nlohmann::json& json, ImuNoise& noise);

} // namespace tl

namespace tl::io {

YAML::Node toYamlNode(const ImuIntrinsics& intrinsics, imu::Type type);
YAML::Node toYamlNode(const ImuNoise& noise, imu::Type type);

} // namespace tl::io

// For yaml-cpp
namespace YAML {

template <>
struct convert<tl::ImuNoise>
{
    static Node encode(const tl::ImuNoise& noise);
    static bool decode(const Node& node, tl::ImuNoise& noise);
};

template <typename T>
struct convert<tl::ImuIntrinsics_<T>>
{
    using _ImuIntrinsics = tl::ImuIntrinsics_<T>;

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
