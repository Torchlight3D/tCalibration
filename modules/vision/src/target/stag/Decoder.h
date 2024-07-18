#pragma once

#include <vector>
#include <bitset>

using Codeword = std::bitset<48>;

class Decoder
{
public:
    Decoder() {}
    Decoder(int hd);
    bool decode(const Codeword& c, int errCorr, int& id, int& shift);

private:
    int wordSize = 48;
    int noOfCodewords;
    std::vector<Codeword> codewords;
};
