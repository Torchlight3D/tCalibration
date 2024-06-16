#include "random.h"

#include <chrono>

namespace tl {

RandomNumberGenerator::RandomNumberGenerator()
    : RandomNumberGenerator(static_cast<size_t>(
          std::chrono::system_clock::now().time_since_epoch().count()))
{
}

RandomNumberGenerator::RandomNumberGenerator(size_t seed)
    : _rng(std::random_device{}())
{
    this->seed(seed);
}

RandomNumberGenerator& RandomNumberGenerator::theRNG(
    const std::optional<size_t>& seed)
{
    static RandomNumberGenerator kRNG;

    if (seed.has_value()) {
        kRNG.seed(seed.value());
    }

    return kRNG;
}

void RandomNumberGenerator::seed(size_t seed) { _rng.seed(seed); }

} // namespace tl
