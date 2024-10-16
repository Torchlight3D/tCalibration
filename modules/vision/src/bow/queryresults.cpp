#include "queryresults.h"

#include <fstream>

namespace tl {

std::ostream& operator<<(std::ostream& os, const Result& ret)
{
    os << "<EntryId: " << ret.Id << ", Score: " << ret.Score << ">";
    return os;
}

std::ostream& operator<<(std::ostream& os, const QueryResults& ret)
{
    if (ret.size() == 1)
        os << "1 result:" << std::endl;
    else
        os << ret.size() << " results:" << std::endl;

    QueryResults::const_iterator rit;
    for (rit = ret.begin(); rit != ret.end(); ++rit) {
        os << *rit;
        if (rit + 1 != ret.end())
            os << std::endl;
    }
    return os;
}

void QueryResults::saveM(const std::string& filename) const
{
    std::ofstream fout{filename};
    for (const auto& result : *this) {
        fout << result.Id << " " << result.Score << std::endl;
    }
}

} // namespace tl
