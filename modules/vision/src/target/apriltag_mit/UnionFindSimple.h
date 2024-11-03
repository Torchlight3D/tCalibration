#pragma once

#include <vector>

namespace AprilTags {

//! Implementation of disjoint set data structure using the union-find algorithm
class UnionFindSimple
{
public:
    explicit UnionFindSimple(int maxId) : data(maxId) { init(); };

    int getSetSize(int thisId) { return data[getRepresentative(thisId)].size; }

    int getRepresentative(int thisId);

    //! Returns the id of the merged node.
    /*  @param aId
     *  @param bId
     */
    int connectNodes(int aId, int bId);

private:
    void init();

    //! Identifies parent ids and sizes.
    struct Data
    {
        int id;
        int size;
    };

    std::vector<Data> data;
};

} // namespace AprilTags
