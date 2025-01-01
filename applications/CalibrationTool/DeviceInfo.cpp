#include "DeviceInfo.h"

namespace tl {
namespace key {
constexpr char kName[]{"name"};
constexpr char kHostAddr[]{"host_address"};
constexpr char kHardwareAddr[]{"hardware_address"};
constexpr char kSN[]{"sn"};
} // namespace key
} // namespace tl

namespace nlohmann {

using namespace tl;

void to_json(json& j, const tl::DeviceInfo& info)
{
    j[key::kName] = info.name;
    j[key::kHostAddr] = info.hostAddr;
    j[key::kHardwareAddr] = info.hardwareAddr;
    j[key::kSN] = info.sn;
}

void from_json(const json& j, tl::DeviceInfo& info)
{
    j.at(key::kName).get_to(info.name);
    j.at(key::kHostAddr).get_to(info.hostAddr);
    j.at(key::kHardwareAddr).get_to(info.hardwareAddr);
    j.at(key::kSN).get_to(info.sn);
}

} // namespace nlohmann

namespace YAML {

using namespace tl;

Emitter& operator<<(Emitter& out, const tl::DeviceInfo& info)
{
    // clang-format off
    out << YAML::BeginMap
        << YAML::Key << key::kName << YAML::Value << YAML::DoubleQuoted << info.name
        << YAML::Key << key::kHostAddr << YAML::Value << YAML::DoubleQuoted << info.hostAddr
        << YAML::Key << key::kHardwareAddr << YAML::Value << YAML::DoubleQuoted << info.hardwareAddr
        << YAML::Key << key::kSN << YAML::Value << YAML::DoubleQuoted << info.sn
        << YAML::EndMap;
    // clang-format on

    return out;
}

Node convert<tl::DeviceInfo>::encode(const tl::DeviceInfo& info)
{
    Node node;
    node[key::kName] = info.name;
    node[key::kHostAddr] = info.hostAddr;
    node[key::kHardwareAddr] = info.hardwareAddr;
    node[key::kSN] = info.sn;

    return node;
}

bool convert<tl::DeviceInfo>::decode(const Node& node, tl::DeviceInfo& info)
{
    // TODO: How to better validate node keys?
    if (!node.IsMap()) {
        return false;
    }

    info.name = node[key::kName].as<std::string>();
    info.hostAddr = node[key::kHostAddr].as<std::string>();
    info.hardwareAddr = node[key::kHardwareAddr].as<std::string>();
    info.sn = node[key::kSN].as<std::string>();

    return true;
}

} // namespace YAML
