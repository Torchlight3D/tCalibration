#include "prosac.h"

#include <set>

#include <tCore/RandomGenerator>

namespace tl {

void ProsacSampler::SetSampleNumber(int k) { kth_sample_number_ = k; }

bool ProsacSampler::Initialize(size_t dataSize)
{
    _dataSize = dataSize;
    kth_sample_number_ = 1;
    return true;
}

bool ProsacSampler::Sample(std::vector<size_t>* indices)
{
    // (Eq. 3)
    auto t_n = 2e4;
    auto n = _minNumSamples;
    for (size_t i{0}; i < _minNumSamples; ++i) {
        t_n *= static_cast<double>(n - i) / (_dataSize - i);
    }

    // Choose min n such that T_n_prime >= t (Eq. 5).
    auto t_n_prime{1.};
    for (int t = 1; t <= kth_sample_number_; t++) {
        if (t > t_n_prime && n < _dataSize) {
            double t_n_plus1 = (t_n * (n + 1.)) / (n + 1. - _minNumSamples);
            t_n_prime += std::ceil(t_n_plus1 - t_n);
            t_n = t_n_plus1;
            n++;
        }
    }

    indices->reserve(_minNumSamples);
    if (t_n_prime < kth_sample_number_) {
        // Randomly sample m data points from the top n data points.
        std::set<size_t> usedIndices;
        for (size_t i{0}; i < _minNumSamples; ++i) {
            size_t index;
            do {
                index = _rng->randInt(size_t{0}, n - 1);
            } while (usedIndices.contains(index));

            usedIndices.emplace(index);
        }

        *indices = {usedIndices.cbegin(), usedIndices.cend()};
    }
    else {
        std::set<size_t> usedIndices;
        // Randomly sample m-1 data points from the top n-1 data points.
        for (size_t i{0}; i < _minNumSamples - 1; ++i) {
            size_t index;
            do {
                index = _rng->randInt(size_t{0}, n - 2);
            } while (usedIndices.contains(index));

            usedIndices.emplace(index);
        }

        *indices = {usedIndices.cbegin(), usedIndices.cend()};

        // Make the last point from the nth position.
        indices->push_back(n);
    }

    CHECK_EQ(indices->size(), _minNumSamples)
        << "Prosac subset is incorrect size!";

    kth_sample_number_++;
    return true;
}

} // namespace tl
