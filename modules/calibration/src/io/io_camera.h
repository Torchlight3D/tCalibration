#pragma once

#include <string>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace tl {

class Camera;

// For nlohmann::json
void to_json(nlohmann::json& json, const Camera& camera);
void from_json(const nlohmann::json& json, Camera& camera);

} // namespace tl

namespace tl::io {

YAML::Node toCamOdoCalYamlNode(const Camera& camera,
                               const std::string& name = "camera");
bool fromCamOdoCalYamlNode(const YAML::Node& node, Camera& camera);

} // namespace tl::io

// For yaml-cpp
namespace YAML {

template <>
struct convert<tl::Camera>
{
    static Node encode(const tl::Camera& camera);
    static bool decode(const Node& node, tl::Camera& camera);
};

} // namespace YAML
