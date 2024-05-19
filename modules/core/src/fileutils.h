#pragma once

#include <filesystem>

namespace tl::file {

std::size_t itemCountUnder(const std::filesystem::path &path);
std::size_t fileCountUnder(const std::filesystem::path &path);
std::size_t dirCountUnder(const std::filesystem::path &path);

double gigabytesAvailable(const std::string &path);

} // namespace tl::file
