#include "util_string.h"

namespace tl {

bool str::Contains(const std::string& str, const std::string& sub)
{
    return str.find(sub) != std::string::npos;
}

}