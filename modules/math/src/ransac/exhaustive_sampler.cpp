#include "exhaustive_sampler.h"

#include <glog/logging.h>

namespace tl {

ExhaustiveSampler::ExhaustiveSampler(
    const std::shared_ptr<RandomNumberGenerator>& rng, int min_num_samples)
    : Sampler(rng, min_num_samples), i_(0), j_(1)
{
    CHECK_EQ(min_num_samples_, 2) << "ExhaustiveSampler makes a hard "
                                     "assumption that the number of "
                                     "samples needed is 2.";
}

bool ExhaustiveSampler::Initialize(int num_datapoints)
{
    CHECK_GE(num_datapoints, min_num_samples_);
    num_datapoints_ = num_datapoints;
    return true;
}

bool ExhaustiveSampler::Sample(std::vector<int>* subset)
{
    subset->emplace_back(i_);
    subset->emplace_back(j_);

    // Increment j and adjust the implicit for loops accordingly.
    ++j_;
    if (j_ >= num_datapoints_) {
        ++i_;
        // If i >= num_datapoints then we have enumerated all possible
        // combinations. We simply reset the outer loop (i) to 0 so that the
        // combinations are rengenerated.
        if (i_ >= num_datapoints_ - 1) {
            i_ = 0;
        }
        j_ = i_ + 1;
    }
    return true;
}

} // namespace tl
