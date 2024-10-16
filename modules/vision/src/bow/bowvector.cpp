#include "bowvector.h"

#include <iostream>
#include <fstream>
#include <cmath>

namespace tl {

void BowVector::addWeight(WordId id, WordValue v)
{
    if (auto vit = lower_bound(id);
        vit != end() && !(key_comp()(id, vit->first))) {
        vit->second += v;
    }
    else {
        this->insert(vit, BowVector::value_type(id, v));
    }
}

void BowVector::addIfNotExist(WordId id, WordValue v)
{
    if (auto vit = this->lower_bound(id);
        vit == this->end() || (this->key_comp()(id, vit->first))) {
        this->insert(vit, BowVector::value_type(id, v));
    }
}

void BowVector::normalize(LNorm norm_type)
{
    double norm = 0.0;

    if (norm_type == L1) {
        for (const auto &[_, val] : *this) {
            norm += std::abs(val);
        }
    }
    else {
        for (const auto &[_, val] : *this) {
            norm += val * val;
        }
        norm = sqrt(norm);
    }

    if (norm > 0.0) {
        for (auto &[_, val] : *this) {
            val /= norm;
        }
    }
}

uint64_t BowVector::getSignature() const
{
    uint64_t sig = 0;
    for (const auto &[id, val] : *this) {
        sig += id + 1e6 * val;
    }
    return sig;
}

std::ostream &operator<<(std::ostream &out, const BowVector &v)
{
    for (const auto &[id, val] : v) {
        out << "<" << id << ", " << val << ">" << ", ";
    }
    return out;
}

void BowVector::saveM(const std::string &filename, size_t W) const
{
    std::fstream f(filename.c_str(), std::ios::out);

    WordId last = 0;
    BowVector::const_iterator bit;
    for (bit = this->begin(); bit != this->end(); ++bit) {
        for (; last < bit->first; ++last) {
            f << "0 ";
        }
        f << bit->second << " ";

        last = bit->first + 1;
    }
    for (; last < (WordId)W; ++last) {
        f << "0 ";
    }

    f.close();
}

void BowVector::toStream(std::ostream &str) const
{
    uint32_t s = size();
    str.write((char *)&s, sizeof(s));
    for (auto d : *this) {
        str.write((char *)&d.first, sizeof(d.first));
        str.write((char *)&d.second, sizeof(d.second));
    }
}

void BowVector::fromStream(std::istream &str)
{
    clear();
    uint32_t s;

    str.read((char *)&s, sizeof(s));
    for (int i = 0; i < s; i++) {
        WordId wid;
        WordValue wv;
        str.read((char *)&wid, sizeof(wid));
        str.read((char *)&wv, sizeof(wv));
        insert(std::make_pair(wid, wv));
    }
}

} // namespace tl
