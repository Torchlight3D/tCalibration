#include "networkutils.h"

#include <format>
#include <regex>

namespace tl {
namespace network {

bool isValidIPv4Address(const std::string& str)
{
    constexpr auto kIPv4Regex{
        R"(^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$)"};
    return !str.empty() && std::regex_match(str, std::regex{kIPv4Regex});
}

bool isValidPort(int port) { return port >= 0 && port <= 65535; }

std::string makeTcpAddress(const std::string& host, int port)
{
    return std::format("tcp://{}:{}", host, port);
}

} // namespace network
} // namespace tl
