#include "random_sampler.h"

#include <numeric>
#include <glog/logging.h>

namespace tl {

bool RandomSampler::Initialize(int num_datapoints)
{
    CHECK_GE(num_datapoints, min_num_samples_);
    sample_indices_.resize(num_datapoints);
    std::iota(sample_indices_.begin(), sample_indices_.end(), 0);
    return true;
}

bool RandomSampler::Sample(std::vector<int>* subset_indices)
{
    subset_indices->reserve(min_num_samples_);
    for (int i{0}; i < min_num_samples_; i++) {
        std::swap(sample_indices_[i],
                  sample_indices_[rng_->RandInt(
                      i, static_cast<int>(sample_indices_.size() - 1))]);
        subset_indices->emplace_back(sample_indices_[i]);
    }

    return true;
}

} // namespace tl
