#pragma once

#include "sampleconsensus.h"

namespace tl {

class ProsacSampler final : public Sampler
{
public:
    using Sampler::Sampler;

    bool Initialize(size_t dataSize) override;

    // NOTE: It's assumed that data is sorted by descending quality order, where
    //       Quality(data[i]) > Quality(data[j]), for all i < j.
    bool Sample(std::vector<size_t>* indices) override;

    // Set the sample such that you are sampling the kth prosac sample (Eq. 6).
    void SetSampleNumber(int k);

private:
    size_t _dataSize;

    // The kth sample of prosac sampling.
    int kth_sample_number_;
};

// Brief:
// Requires the data to be in sorted order by quality (with highest quality at
// index 0).
//
// Ref:
// "Matching with PROSAC - Progressive Sampling Consensus" by Chum and Matas.
template <class ModelEstimator>
class Prosac : public SampleConsensus<ModelEstimator>
{
    using Base = SampleConsensus<ModelEstimator>;

public:
    using Base::Base;

    bool Initialize() override
    {
        auto* sampler =
            new ProsacSampler(this->_estimator.SampleSize(), this->_params.rng);
        return Base::SetUpSampler(sampler);
    }
};

} // namespace tl
