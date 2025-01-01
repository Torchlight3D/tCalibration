#include "TaskInfo.h"

#define UUID_SYSTEM_GENERATOR
#include <stduuid/uuid.h>

namespace tl {

TaskInfo::TaskInfo(const std::string& uuid)
    : uuid(uuid.empty() ? uuids::to_string(uuids::uuid_system_generator{}())
                        : uuid)
{
}

namespace key {
constexpr char kUuid[]{"uuid"};
constexpr char kDateTime[]{"datetime"};
constexpr char kTemperature[]{"temperature"};
} // namespace key

} // namespace tl

namespace nlohmann {

using namespace tl;

void to_json(json& j, const tl::TaskInfo& info)
{
    j[key::kUuid] = info.uuid;
    j[key::kDateTime] = info.datetime;
    j[key::kTemperature] = {info.minTemp, info.maxTemp};
}

void from_json(const json& j, tl::TaskInfo& info)
{
    j.at(key::kUuid).get_to(info.uuid);
    j.at(key::kDateTime).get_to(info.datetime);
    std::pair<double, double> minmaxTemp;
    j.at(key::kTemperature).get_to(minmaxTemp);
    info.minTemp = minmaxTemp.first;
    info.maxTemp = minmaxTemp.second;
}

} // namespace nlohmann

namespace YAML {

using namespace tl;

Emitter& operator<<(Emitter& out, const tl::TaskInfo& info)
{
    // clang-format off
    out << YAML::BeginMap
        << YAML::Key << key::kUuid << YAML::Value << YAML::DoubleQuoted << info.uuid
        << YAML::Key << key::kDateTime << YAML::Value << YAML::DoubleQuoted << info.datetime
        << YAML::Key << key::kTemperature << YAML::Value << YAML::Flow
        << YAML::BeginSeq << info.minTemp << info.maxTemp << YAML::EndSeq
        << YAML::EndMap;
    // clang-format on

    return out;
}

Node convert<tl::TaskInfo>::encode(const tl::TaskInfo& info)
{
    Node node;
    node[key::kUuid] = info.uuid;
    node[key::kDateTime] = info.datetime;
    node[key::kTemperature].push_back(info.minTemp);
    node[key::kTemperature].push_back(info.maxTemp);

    return node;
}

bool convert<tl::TaskInfo>::decode(const Node& node, tl::TaskInfo& info)
{
    if (!node.IsMap()) {
        return false;
    }

    info.uuid = node[key::kUuid].as<std::string>();
    info.datetime = node[key::kDateTime].as<std::string>();
    info.minTemp = node[key::kTemperature][0].as<double>();
    info.maxTemp = node[key::kTemperature][1].as<double>();

    return true;
}

} // namespace YAML
