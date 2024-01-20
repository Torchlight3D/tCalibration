#pragma once

#include <vector>

#include "sampler.h"

namespace tl {

// This is guaranteed to generate a unique sample by
// performing a Fisher-Yates sampling.
class RandomSampler : public Sampler
{
public:
    using Sampler::Sampler;

    bool Initialize(int num_datapoints) override;

    // Samples the input variable data and fills the vector subset with the
    // random samples.
    bool Sample(std::vector<int>* subset_indices) override;

private:
    std::vector<int> sample_indices_;
};

} // namespace tl
