#pragma once

#include <vector>

namespace tl {

struct TagCodes
{
    std::vector<unsigned long long> codes;
    int bits;
    int minHammingDistance;

    TagCodes(int bits, int minHammingDistance, const unsigned long long* codesA,
             size_t num)
        : codes(codesA, codesA + num),
          bits(bits),
          minHammingDistance(minHammingDistance)

    {
    }
};

} // namespace tl
