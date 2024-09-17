#pragma once

#include <vector>
#include <bitset>

namespace tl {
namespace stag {

using Codeword = std::bitset<48>;

class Decoder
{
public:
    Decoder() {}
    explicit Decoder(int hd);

    bool decode(const Codeword& c, int errCorr, int& id, int& shift);

private:
    int wordSize = 48;
    int noOfCodewords;
    std::vector<Codeword> codewords;
};

} // namespace stag
} // namespace tl
