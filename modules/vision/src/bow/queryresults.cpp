#include "queryresults.h"

#include <fstream>

namespace tl {

void QueryResults::saveM(const std::string& filename) const
{
    std::ofstream fout{filename};
    for (const auto& result : *this) {
        fout << result.Id << " " << result.Score << std::endl;
    }
}

std::ostream& operator<<(std::ostream& os, const Result& res)
{
    os << "{EntryId: " << res.Id << ", Score: " << res.Score << "}";
    return os;
}

std::ostream& operator<<(std::ostream& out, const QueryResults& results)
{
    out << "Result (" << results.size() << "):\n";

    for (const auto& res : results) {
        out << res << "\n";
    }
    return out;
}

} // namespace tl
