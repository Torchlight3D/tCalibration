#include "fileutils.h"

namespace tl::file {

namespace fs = std::filesystem;

namespace internal {

using Filter = std::add_pointer_t<bool(const fs::path &)>;
inline std::size_t countUnder(const fs::path &path, Filter filter)
{
    return static_cast<size_t>(std::count_if(fs::directory_iterator{path},
                                             fs::directory_iterator{}, filter));
}

} // namespace internal

std::size_t itemCountUnder(const fs::path &path)
{
    return std::distance(fs::directory_iterator{path},
                         fs::directory_iterator{});
}

std::size_t fileCountUnder(const fs::path &path)
{
    return internal::countUnder(path, fs::is_regular_file);
}

std::size_t dirCountUnder(const fs::path &path)
{
    return internal::countUnder(path, fs::is_directory);
}

double gigabytesAvailable(const std::string &path)
{
    std::error_code fileError;
    const auto spaceInfo = fs::space(fs::path{path}, fileError);
    constexpr double b2Gb = 1. / 1024. / 1024. / 1024.;
    return fileError ? 0. : (spaceInfo.available * b2Gb);
}

} // namespace tl::file
