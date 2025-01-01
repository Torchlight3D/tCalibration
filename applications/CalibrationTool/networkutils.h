#pragma once

#include <string>

namespace tl {
namespace network {

bool isValidIPv4Address(const std::string& address);

// We have ports (uint16):
// 1. Well-known ports range: 0-1023
// 2. Registered ports range: 1024-49151
// 3. Dynamic ports (a.k.a private ports) range: 49152-65535
bool isValidPort(int port);

std::string makeTcpAddress(const std::string& host, int port);

} // namespace network
} // namespace tl
