#pragma once

#if _WIN32
#include <cstdint>
#endif
#include <map>
#include <string>

namespace tl {

using WordId = unsigned int;

using WordValue = double;

/// Id of nodes in the vocabulary tree
using NodeId = unsigned int;

enum LNorm
{
    L1,
    L2
};

enum WeightingType
{
    TF_IDF,
    TF,
    IDF,
    BINARY
};

enum ScoringType
{
    L1_NORM,
    L2_NORM,
    CHI_SQUARE,
    KL,
    BHATTACHARYYA,
    DOT_PRODUCT
};

/// Vector of words to represent images
class BowVector : public std::map<WordId, WordValue>
{
public:
    /**
     * Adds a value to a word value existing in the vector, or creates a new
     * word with the given value
     * @param id word id to look for
     * @param v value to create the word with, or to add to existing word
     */
    void addWeight(WordId id, WordValue v);

    /**
     * Adds a word with a value to the vector only if this does not exist yet
     * @param id word id to look for
     * @param v value to give to the word if this does not exist
     */
    void addIfNotExist(WordId id, WordValue v);

    /**
     * L1-Normalizes the values in the vector
     * @param norm_type norm used
     */
    void normalize(LNorm norm_type);

    // returns a unique number from the configuration
    uint64_t getSignature() const;

    // serialization
    // saveToMat
    void saveM(const std::string &filename, size_t numWords) const;
    void toStream(std::ostream &str) const;
    void fromStream(std::istream &str);

    friend std::ostream &operator<<(std::ostream &out, const BowVector &v);
};

} // namespace tl
