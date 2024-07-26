#pragma once

#include <vector>

#include <NTL/lzz_pE.h>
#include <NTL/lzz_pX.h>
#include <NTL/lzz_pE.h>
#include <NTL/lzz_pEX.h>

namespace cv {
namespace runetag {

class Coding
{
public:
    Coding();
    void init();

    long get_index(const std::vector<long>& code_vec);
    void align(std::vector<long>& code_vec, long& index, long& rotation);
    int decode(std::vector<long>& code_vec);
    long generate(std::vector<long>& code, long index);

    static std::vector<bool> unpack(const std::vector<long> code);
    static std::vector<long> pack(const std::vector<bool> code);

private:
    void Euclidean(NTL::zz_pEX& key, NTL::zz_pEX& locator, NTL::zz_pEX zeta,
                   long max_deg);
    void sft(std::vector<long>& out, const std::vector<long>& in);
    void isft(std::vector<long>& out, const std::vector<long>& in);

    const long start_range;
    const long end_range;
    const long code_length;
    const long num_words;
    std::vector<NTL::zz_pE> alpha;

    const long align_k;
    const long align_p;
    const long align_root;
    const long align_logn1;
    std::vector<long> align_pow;
    std::vector<long> align_log;
};

} // namespace runetag
} // namespace cv
