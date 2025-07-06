#pragma once

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace tl {

struct TaskInfo
{
    std::string uuid;
    std::string datetime{};
    double minTemp{0.}, maxTemp{0.};

    explicit TaskInfo(const std::string& uuid = {});
};

} // namespace tl

namespace nlohmann {

void to_json(json& j, const tl::TaskInfo& info);
void from_json(const json& j, tl::TaskInfo& info);

} // namespace nlohmann

namespace YAML {

Emitter& operator<<(Emitter& out, const tl::TaskInfo& info);

template <>
struct convert<tl::TaskInfo>
{
    static Node encode(const tl::TaskInfo& info);
    static bool decode(const Node& node, tl::TaskInfo& info);
};

} // namespace YAML
