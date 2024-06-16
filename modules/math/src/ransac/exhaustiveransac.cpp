#include "exhaustiveransac.h"

namespace tl {

ExhaustiveSampler::ExhaustiveSampler(size_t minSampleSize,
                                     std::shared_ptr<RandomNumberGenerator> rng)
    : Sampler(minSampleSize, rng), _i(0), _j(1)
{
    CHECK_EQ(_minNumSamples, 2) << "ExhaustiveSampler makes a hard "
                                   "assumption that the number of "
                                   "samples needed is 2.";
}

bool ExhaustiveSampler::Initialize(size_t dataSize)
{
    if (dataSize < _minNumSamples) {
        return false;
    }

    _dataSize = dataSize;
    return true;
}

bool ExhaustiveSampler::Sample(std::vector<size_t>* indices)
{
    indices->emplace_back(_i);
    indices->emplace_back(_j);

    // We generate combinations by essentially iterating over the nested loops:
    //   for (int i = 0; i < num_datapoints; i++) {
    //     for (int j = i + 1; j < num_datapoints; j++) {
    //       sample = (i, j);
    //     }
    //   }
    //

    ++_j;
    if (_j >= _dataSize) {
        ++_i;
        // If i >= num_datapoints then we have enumerated all possible
        // combinations. We simply reset the outer loop (i) to 0 so that the
        // combinations are rengenerated.
        if (_i >= _dataSize - 1) {
            _i = 0;
        }
        _j = _i + 1;
    }

    return true;
}

} // namespace tl
