#pragma once

#include <string>

#include <json/json.hpp>
#include <yaml-cpp/yaml.h>

namespace thoht {

class Camera;

// For nlohmann::json
void to_json(nlohmann::json& json, const Camera& camera);
void from_json(const nlohmann::json& json, Camera& camera);

} // namespace thoht

namespace thoht::io {

// For Luba Vio module
YAML::Node toCamOdoCalYamlNode(const Camera& camera,
                               const std::string& name = "camera");
bool fromCamOdoCalYamlNode(const YAML::Node& node, Camera& camera);

} // namespace thoht::io

// For yaml-cpp
namespace YAML {

template <>
struct convert<thoht::Camera>
{
    static Node encode(const thoht::Camera& camera);
    static bool decode(const Node& node, thoht::Camera& camera);
};

} // namespace YAML
