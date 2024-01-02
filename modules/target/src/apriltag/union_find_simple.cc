#include "union_find_simple.h"

#include <iostream>

#include <glog/logging.h>

namespace apriltags {

UnionFindSimple::UnionFindSimple(int maxId) : _data(maxId) { init(); }

int UnionFindSimple::getRepresentative(int thisId)
{
    // terminal case: a node is its own parent
    if (_data[thisId].id == thisId) {
        return thisId;
    }

    // otherwise, recurse...
    int root = getRepresentative(_data[thisId].id);

    // short circuit the path
    _data[thisId].id = root;

    return root;
}

int UnionFindSimple::connectNodes(int aId, int bId)
{
    int aRoot = getRepresentative(aId);
    int bRoot = getRepresentative(bId);

    if (aRoot == bRoot)
        return aRoot;

    int asz = _data[aRoot].size;
    int bsz = _data[bRoot].size;

    if (asz > bsz) {
        _data[bRoot].id = aRoot;
        _data[aRoot].size += bsz;
        return aRoot;
    }

    _data[aRoot].id = bRoot;
    _data[bRoot].size += asz;
    return bRoot;
}

void UnionFindSimple::printDataVector() const
{
    for (size_t i{0}; i < _data.size(); i++) {
        LOG(INFO) << "data[" << i << "]: "
                  << " id:" << _data[i].id << " size:" << _data[i].size;
    }
}

void UnionFindSimple::init()
{
    for (size_t i{0}; i < _data.size(); ++i) {
        // everyone is their own cluster of size 1
        _data[i].id = i;
        _data[i].size = 1;
    }
}

} // namespace apriltags
