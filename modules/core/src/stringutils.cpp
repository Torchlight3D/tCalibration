#include "stringutils.h"

namespace tl::str {

bool Contains(const std::string& str, const std::string& sub)
{
    return str.find(sub) != std::string::npos;
}

}