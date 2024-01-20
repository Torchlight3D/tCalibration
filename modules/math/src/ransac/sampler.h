#pragma once

#include <memory>
#include <vector>

#include <tMath/RandomGenerator>

namespace tl {

class Sampler
{
public:
    Sampler(const std::shared_ptr<RandomNumberGenerator>& rng,
            int min_num_samples);
    virtual ~Sampler() = default;

    // Initializes any non-trivial variables and sets up sampler if
    // necessary. Must be called before Sample is called.
    virtual bool Initialize(int num_datapoints) = 0;

    // Samples the input variable data and fills the vector subset with the
    // samples.
    virtual bool Sample(std::vector<int>* subset_indices) = 0;

protected:
    std::shared_ptr<RandomNumberGenerator> rng_;
    int min_num_samples_;
};

inline Sampler::Sampler(const std::shared_ptr<RandomNumberGenerator>& rng,
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
