#pragma once

#include <vector>

namespace apriltags {

//! Implementation of disjoint set data structure using the union-find algorithm
class UnionFindSimple
{
public:
    explicit UnionFindSimple(int maxId);

    int getRepresentative(int thisId);

    inline int getSetSize(int thisId)
    {
        return _data[getRepresentative(thisId)].size;
    }

    //! Returns the id of the merged node.
    /*  @param aId
     *  @param bId
     */
    int connectNodes(int aId, int bId);

    void printDataVector() const;

private:
    void init();

private:
    struct Data
    {
        int id;
        int size;
    };
    std::vector<Data> _data;
};

} // namespace apriltags
