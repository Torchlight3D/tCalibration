#include "sampler.h"

#include <numeric>

#include <tCore/RandomGenerator>

namespace tl {

Sampler::Sampler(size_t minNumSamples,
                 std::shared_ptr<RandomNumberGenerator> rng)
    : _minNumSamples(minNumSamples)
{
    if (!rng) {
        _rng = std::make_shared<RandomNumberGenerator>();
    }
    else {
        _rng = rng;
    }
}

Sampler::~Sampler() = default;

bool RandomSampler::Initialize(size_t dataSize)
{
    if (dataSize <= _minNumSamples) {
        return false;
    }

    _indices.resize(dataSize);
    std::iota(_indices.begin(), _indices.end(), size_t{0});
    return true;
}

bool RandomSampler::Sample(std::vector<size_t>* indices)
{
    indices->reserve(_minNumSamples);
    for (size_t i{0}; i < _minNumSamples; ++i) {
        std::swap(_indices[i], _indices[_rng->randInt(i, _indices.size() - 1)]);
        indices->emplace_back(_indices[i]);
    }

    return true;
}

} // namespace tl
