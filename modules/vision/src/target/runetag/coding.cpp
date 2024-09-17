#include "coding.h"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>

#include <NTL/lzz_pE.h>
#include <NTL/lzz_pX.h>
#include <NTL/lzz_pE.h>
#include <NTL/lzz_pEX.h>

namespace tl {
namespace runetag {

namespace {
// Computer the i-th syndrome of code
// ai is the i-th power of the generating root of unity a
inline void Syndrome(NTL::zz_pE& S, const NTL::zz_pX code, const NTL::zz_pE ai)
{
    NTL::CompMod(S.LoopHole(), code, NTL::_zz_pE__rep(ai),
                 NTL::zz_pE::modulus());
}

constexpr long generator[43] = {1, 1, 6, 4, 6, 0, 3, 1, 5, 3, 5, 4, 0, 4, 6,
                                3, 4, 6, 3, 6, 4, 3, 6, 4, 0, 4, 5, 3, 5, 1,
                                3, 0, 6, 4, 6, 1, 1, 0, 0, 0, 0, 0, 0};

void coding_error_callback(void) { throw std::invalid_argument("NTL error"); }

} // namespace

class Coding::Impl
{
public:
    Impl();

    void sft(std::vector<long>& out, const std::vector<long>& in);
    void isft(std::vector<long>& out, const std::vector<long>& in);

    void Euclidean(NTL::zz_pEX& key, NTL::zz_pEX& locator, NTL::zz_pEX zeta,
                   long max_deg);

public:
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

Coding::Impl::Impl()
    : start_range(8),
      end_range(36),
      code_length(43),
      num_words(117649),
      align_k(4),
      align_p(173),
      align_root(2),
      align_logn1(89)
{
}

void Coding::Impl::sft(std::vector<long>& out, const std::vector<long>& in)
{
    for (long i = 0; i != code_length; ++i) {
        out[i] = 0;
        const long strobe = (align_k * i) % (align_p - 1);
        for (long j = 0; j != code_length; ++j) {
            out[i] += align_pow[(strobe * j) % (align_p - 1)] * in[j];
            out[i] = out[i] % align_p;
            // std::cerr << align_pow[(strobe*j)%(align_p-1)] << " ";
        }
    }
}

void Coding::Impl::isft(std::vector<long>& out, const std::vector<long>& in)
{
    for (long i = 0; i != code_length; ++i) {
        out[i] = 0;
        const long strobe = (align_k * i) % (align_p - 1);
        for (long j = 0; j != code_length; ++j) {
            const long psn = (strobe * j) % (align_p - 1);
            out[i] +=
                align_pow[(align_logn1 + align_p - 2 - psn) % (align_p - 1)] *
                in[j];
            out[i] = out[i] % align_p;
            // std::cerr << align_pow[( align_logn1+align_p-2-psn )%(align_p-1)]
            // << " ";
        }
    }
}

void Coding::Impl::Euclidean(NTL::zz_pEX& key, NTL::zz_pEX& locator,
                             NTL::zz_pEX zeta, long max_deg)
{
    NTL::zz_pEX U(0, 1L), V(0, 0L), K, M, N, Q, R;
    key = NTL::zz_pEX(end_range - start_range, 1L);
    while (NTL::deg(key) > max_deg) {
        NTL::DivRem(Q, R, key, zeta);
        M = locator - U * Q;
        N = K - V * Q;
        key = zeta;
        zeta = R;
        locator = U;
        K = V;
        U = M;
        V = N;
    }
}

Coding::Coding() : d(std::make_unique<Impl>()) {}

Coding::~Coding() = default;

void Coding::init()
{
    // initialize finite field Z_7^6
    NTL::zz_p::init(7);
    NTL::zz_pX P = NTL::zz_pX(6, 1) + NTL::zz_pX(5, 6) + NTL::zz_pX(3, 2) +
                   NTL::zz_pX(1, 6) + 1L; // P irreducible polynomial
    NTL::zz_pE::init(P);

    // root of unity generating the generalized BCH code
    NTL::zz_pX wx = NTL::zz_pX(5, 1) + NTL::zz_pX(4, 4) + NTL::zz_pX(2, 5) +
                    NTL::zz_pX(1, 6);
    NTL::zz_pE w, wi; //=power(w,start_range);
    NTL::conv(w, wx);
    NTL::set(wi);
    d->alpha.resize(d->code_length + 1);
    for (long i = 0; i != d->code_length + 1; ++i) {
        d->alpha[i] = wi;
        wi *= w;
    }

    d->align_pow.resize(d->align_p);
    d->align_log.resize(d->align_p);
    long a = 1L;
    for (long i = 0; i != d->align_p - 1; ++i) {
        d->align_pow[i] = a;
        d->align_log[a] = i;
        a = (a * d->align_root) % d->align_p;
    }

    NTL::ErrorCallback = coding_error_callback;
}

long Coding::get_index(const std::vector<long>& code_vec)
{
    long index = 0;
    index +=
        (5 * code_vec[0] + 2 * code_vec[3] + 6 * code_vec[4] + code_vec[5]) % 7;
    index *= 7;
    index += (2 * code_vec[2] + 6 * code_vec[3] + code_vec[4]) % 7;
    index *= 7;
    index += (2 * code_vec[1] + 6 * code_vec[2] + code_vec[3]) % 7;
    index *= 7;
    index += (2 * code_vec[0] + 6 * code_vec[1] + code_vec[2]) % 7;
    index *= 7;
    index += (6 * code_vec[0] + code_vec[1]) % 7;
    index *= 7;
    index += code_vec[0];

    return index;
}

void Coding::align(std::vector<long>& code_vec, long& index, long& rotation)
{
    std::vector<long> FT(d->code_length);
    d->sft(FT, code_vec);

    if (FT[1] == 0)
        throw std::invalid_argument("periodic code");

    rotation = d->align_log[FT[1]] / d->align_k;
    const long rot_idx = d->align_p - 1 - d->align_k * rotation;
    for (long i = 1; i != d->code_length; ++i) {
        FT[i] = (FT[i] * d->align_pow[(rot_idx * i) % (d->align_p - 1)]) %
                d->align_p;
    }

    d->isft(code_vec, FT);
    index = get_index(code_vec);
}

int Coding::decode(std::vector<long>& code_vec)
{
    NTL::zz_pX code; // received code
    code.rep.SetLength(d->code_length);
    NTL::zz_pEX Ex(0, 1L); // erasure polynomial
    long erasure_num = 0;
    for (long i = 0;
         i != std::min(d->code_length, static_cast<long>(code_vec.size()));
         ++i) {
        if (code_vec[i] >= 0 && code_vec[i] < 7)
            code.rep[i] = code_vec[i];
        else {
            code.rep[i] = 0L;
            ++erasure_num;
            Ex *= NTL::zz_pEX(1, -d->alpha[i]) + 1;
        }
    }
    code.normalize();
    const long t = (d->end_range - d->start_range - erasure_num) / 2;

    NTL::zz_pEX Sx; // syndrome polynomial
    Sx.rep.SetLength(d->end_range - d->start_range);
    for (long i = 0; i != d->end_range - d->start_range; ++i) {
        Syndrome(Sx.rep[i], code, d->alpha[d->start_range + i]);
    }
    Sx.normalize();
    code.rep.SetLength(d->code_length);

    if (!NTL::IsZero(Sx)) {
        NTL::zz_pEX O, Lx, DLx;

        // Euclidean algorithm
        Euclidean(O, Lx, NTL::MulTrunc(Ex, Sx, d->end_range - d->start_range),
                  t + erasure_num);
        // O: key (error-evaluator) polynomial
        // Lx: error locator polynomial

        // std::cerr << Lx << "\n" << Ex << "\n";
        // Forney's algorithm
        Lx *= Ex;
        NTL::diff(DLx, Lx);
        // Lx: error+erasure locator polynomial
        // DLx: symbolic derivative of Lx

        // locate errors
        for (long pos = 0; pos != d->code_length; ++pos) {
            if (NTL::IsZero(eval(Lx, d->alpha[d->code_length - pos]))) {
                // error or erasure
                NTL::zz_pE e =
                    d->alpha[d->code_length -
                             (pos * (d->start_range - 1)) % d->code_length] *
                    eval(O, d->alpha[d->code_length - pos]) /
                    eval(DLx, d->alpha[d->code_length - pos]);
                code.rep[pos] += _zz_pE__rep(e).rep[0];

                // std::cerr << "error/erasure at position " << pos << " val: "
                // << -e << std::endl;
            }
        }
    }

    code_vec.resize(d->code_length);
    for (long i = 0; i != d->code_length; ++i) {
        code_vec[i] = NTL::rep(code.rep[i]);
    }

    for (long i = 0; i != d->end_range - d->start_range; ++i) {
        Syndrome(Sx.rep[i], code, d->alpha[d->start_range + i]);
    }

    Sx.normalize();
    if (!NTL::IsZero(Sx)) {
        /*
        std::cerr << "Uncorrectable code\n";
        std::cerr << code << std::endl;
        std::cerr << Sx << std::endl;
        exit(2);
        */
        return 1;
    }
    return 0;
}

long Coding::generate(std::vector<long>& code, long index)
{
    index %= d->num_words;

    code.resize(d->code_length);
    for (long i = 0; i != d->code_length; ++i) {
        code[i] = 0L;
    }

    long start = 0L;
    while (index) {
        const long val = index % 7;
        for (long i = 0; i != d->code_length; ++i)
            code[(start + i) % d->code_length] += (val * generator[i]) % 7;
        index /= 7;
        ++start;
    }
    for (long i = 0; i != d->code_length; ++i) {
        code[i] %= 7;
    }

    align(code, index, start);
    return index;
}

std::vector<bool> Coding::unpack(const std::vector<long> code)
{
    std::vector<bool> bitcode(code.size() * 3);

    size_t idx = 0;
    for (std::vector<long>::const_iterator it = code.begin(); it != code.end();
         ++it) {
        const long& c = *it + 1; // unpacked code should go from 1 to 7 as 0 is
                                 // reserved to erasures
        if (c > 7 || c < 1)
            throw std::invalid_argument("Invalid code");

        bitcode[idx++] = ((c / 4) % 2);
        bitcode[idx++] = ((c / 2) % 2);
        bitcode[idx++] = ((c) % 2);
    }
    return bitcode;
}

std::vector<long> Coding::pack(const std::vector<bool> bitcode)
{
    if (bitcode.size() % 3 != 0)
        throw std::invalid_argument("Wrong code length");
    std::vector<long> code(bitcode.size() / 3);

    size_t idx = 0;
    size_t cidx = 0;
    while (idx < bitcode.size()) {
        code[cidx] = (long)(bitcode[idx]) * 4 + (long)(bitcode[idx + 1]) * 2 +
                     (long)(bitcode[idx + 2]);
        code[cidx]--;

        idx += 3;
        cidx++;
    }

    return code;
}

} // namespace runetag
} // namespace tl
