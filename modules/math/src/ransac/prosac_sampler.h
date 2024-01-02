#pragma once

#include "sampler.h"

namespace thoht {

class ProsacSampler : public Sampler
{
public:
    using Sampler::Sampler;

    bool Initialize(int num_datapoints) override;

    // Samples the input variable data and fills the vector subset with the
    // prosac samples. NOTE: This assumes that data is in sorted order by
    // quality where data[i] is of higher quality than data[j] for all i < j.
    bool Sample(std::vector<int>* subset_indices) override;

    // Set the sample such that you are sampling the kth prosac sample (Eq. 6).
    void SetSampleNumber(int k);

private:
    int num_datapoints_;
    // Number of iterations of PROSAC before it just acts like ransac.
    int ransac_convergence_iterations_;

    // The kth sample of prosac sampling.
    int kth_sample_number_;
};

} // namespace thoht
