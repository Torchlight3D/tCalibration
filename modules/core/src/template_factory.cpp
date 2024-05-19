#include "template_factory.h"

#if defined(_MSC_VER)
#include <Windows.h>
#include <DbgHelp.h>
#else
#include <cxxabi.h>
#endif

namespace tl {

// NOTE: Refer to the answer from
// [stackoverflow](https://stackoverflow.com/questions/281818/unmangling-the-result-of-stdtype-infoname)
std::string demangle(const char* name)
{
    using StrDeleter = std::add_pointer_t<void(void*)>;

#if defined(_MSC_VER)
    // TODO: Use C++ style
    char* realname = (char*)malloc(1024 * sizeof(char));
    if (realname) {
        realname[0] = 0;
        ::UnDecorateSymbolName(name, realname, 1024, 0);
    }

    const auto res =
        std::string{realname && realname[0] ? realname : ""}.substr(
            std::string{"class "}.size());
    free(realname);
    return res;
#else
    // Initialize with random number
    int status{-4};

    std::unique_ptr<char, StrDeleter> res{
        abi::__cxa_demangle(name, nullptr, nullptr, &status), std::free};
    return (status == 0) ? res.get() : name;
#endif
}

} // namespace tl
