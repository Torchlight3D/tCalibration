#include "sampler.h"

namespace tl {

Sampler::Sampler(const std::shared_ptr<RandomNumberGenerator>& rng,
                 int min_num_samples)
    : min_num_samples_(min_num_samples)
{
    if (!rng) {
        rng_ = std::make_shared<RandomNumberGenerator>();
    }
    else {
        rng_ = rng;
    }
}

} // namespace tl
