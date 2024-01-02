#include "tag_family.h"

#include <iostream>

#include <glog/logging.h>

#include "tag_detection.h"

/**

// example of instantiation of tag family:

#include "TagFamily.h"
#include "Tag36h11.h"
TagFamily *tag36h11 = new TagFamily(tagCodes36h11);

// available tag families:

#include "Tag16h5.h"
#include "Tag16h5_other.h"
#include "Tag25h7.h"
#include "Tag25h9.h"
#include "Tag36h11.h"
#include "Tag36h11_other.h"
#include "Tag36h9.h"

*/

namespace apriltags {

unsigned char TagFamily::popCountTable[TagFamily::popCountTableSize];

TagFamily::TableInitializer TagFamily::initializer;

TagFamily::TagFamily(const TagCodes& tagCodes, size_t blackBorder)
    : blackBorder(blackBorder),
      bits(tagCodes.bits),
      dimension((int)std::sqrt((float)bits)),
      minimumHammingDistance(tagCodes.minHammingDistance),
      errorRecoveryBits(1),
      codes()
{
    if (bits != dimension * dimension) {
        LOG(ERROR) << "Error: TagFamily constructor called with bits=" << bits
                   << "; must be a square number!";
    }
    codes = tagCodes.codes;
}

void TagFamily::setErrorRecoveryBits(int b) { errorRecoveryBits = b; }

void TagFamily::setErrorRecoveryFraction(float v)
{
    errorRecoveryBits =
        static_cast<int>(((minimumHammingDistance - 1) / 2) * v);
}

size_t TagFamily::rotate90(size_t w, int d)
{
    constexpr size_t kOne{1};

    size_t wr = 0;
    for (int r = d - 1; r >= 0; r--) {
        for (int c = 0; c < d; c++) {
            int b = r + d * c;
            wr = wr << 1;

            if ((w & (kOne << b)) != 0)
                wr |= 1;
        }
    }

    return wr;
}

int TagFamily::hammingDistance(size_t a, size_t b) { return popCount(a ^ b); }

unsigned char TagFamily::popCountReal(size_t w)
{
    unsigned char cnt = 0;
    while (w != 0) {
        w &= (w - 1);
        ++cnt;
    }

    return cnt;
}

int TagFamily::popCount(size_t w)
{
    int count = 0;
    while (w != 0) {
        count += popCountTable[(unsigned int)(w & (popCountTableSize - 1))];
        w >>= popCountTableShift;
    }
    return count;
}

void TagFamily::decode(TagDetection& det, size_t rCode) const
{
    size_t rCodes[4];
    rCodes[0] = rCode;
    rCodes[1] = rotate90(rCodes[0], dimension);
    rCodes[2] = rotate90(rCodes[1], dimension);
    rCodes[3] = rotate90(rCodes[2], dimension);

    int bestId = -1;
    int bestHamming = INT_MAX;
    int bestRotation = 0;
    size_t bestCode = 0;
    for (size_t id{0}; id < codes.size(); id++) {
        for (int rot{0}; rot < 4; rot++) {
            const int dist = hammingDistance(rCodes[rot], codes[id]);
            if (dist < bestHamming) {
                bestHamming = dist;
                bestRotation = rot;
                bestId = id;
                bestCode = codes[id];
            }
        }
    }

    det.id = bestId;
    det.hammingDistance = bestHamming;
    det.rotation = bestRotation;
    det.good = (det.hammingDistance <= errorRecoveryBits);
    det.obsCode = rCode;
    det.code = bestCode;
}

void TagFamily::printHammingDistances() const
{
    std::vector<int> hammings(dimension * dimension + 1);
    for (size_t i{0}; i < codes.size(); i++) {
        size_t r0 = codes[i];
        size_t r1 = rotate90(r0, dimension);
        size_t r2 = rotate90(r1, dimension);
        size_t r3 = rotate90(r2, dimension);
        for (size_t j{i + 1}; j < codes.size(); j++) {
            int d = std::min(std::min(hammingDistance(r0, codes[j]),
                                      hammingDistance(r1, codes[j])),
                             std::min(hammingDistance(r2, codes[j]),
                                      hammingDistance(r3, codes[j])));
            hammings[d]++;
        }
    }

    for (size_t i{0}; i < hammings.size(); i++) {
        LOG(INFO) << "hammings: " << i << " = " << hammings[i];
    }
}

} // namespace apriltags
