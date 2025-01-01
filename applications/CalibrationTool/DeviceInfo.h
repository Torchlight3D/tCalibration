#pragma once

#include <yaml-cpp/yaml.h>

#include <json/json.hpp>

namespace tl {

struct DeviceInfo
{
    std::string name{};
    std::string hostAddr{};
    std::string hardwareAddr{};
    std::string sn{};
    bool isVirtual{true};
};

} // namespace tl

namespace nlohmann {

void to_json(json& j, const tl::DeviceInfo& info);
void from_json(const json& j, tl::DeviceInfo& info);

} // namespace nlohmann

namespace YAML {

// NOTE: Workaround to used with OpenCV yaml.
// OpenCV yaml can't parse not quoted string as yaml value, while yaml-cpp only
// offers "DoubleQuoted" option in emit style interface (???)
Emitter& operator<<(Emitter& out, const tl::DeviceInfo& info);

template <>
struct convert<tl::DeviceInfo>
{
    static Node encode(const tl::DeviceInfo& info);
    static bool decode(const Node& node, tl::DeviceInfo& info);
};

} // namespace YAML
