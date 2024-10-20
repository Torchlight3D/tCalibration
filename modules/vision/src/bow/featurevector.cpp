#include "featurevector.h"

#include <iostream>

namespace tl {

void FeatureVector::addFeature(NodeId id, unsigned int i_feature)
{
    if (auto low = this->lower_bound(id);
        low != this->end() && low->first == id) {
        low->second.push_back(i_feature);
    }
    else {
        low = this->insert(low, {id, std::vector<unsigned int>()});
        low->second.push_back(i_feature);
    }
}

std::ostream &operator<<(std::ostream &out, const FeatureVector &v)
{
    if (v.empty()) {
        return out;
    }

    // Example:
    // <NodeId>: [index0 index1 index2 ...]

    for (const auto &[nodeId, indices] : v) {
        out << "<" << nodeId << ">: [";
        for (const auto &index : indices) {
            out << index << " ";
        }
        out << "]\n";
    }

    return out;
}

} // namespace tl
