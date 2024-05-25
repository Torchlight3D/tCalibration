#pragma once

#include "sampleconsensus.h"
#include "prosac_sampler.h"

namespace tl {

// Estimate a model using PROSAC. The Estimate method is inherited, but for
// PROSAC requires the data to be in sorted order by quality (with highest
// quality at index 0).
template <class ModelEstimator>
class Prosac : public SampleConsensusEstimator<ModelEstimator>
{
    using Base = SampleConsensusEstimator<ModelEstimator>;

public:
    using Data = typename ModelEstimator::Data;
    using Model = typename ModelEstimator::Model;

    using Base::Base;

    bool Initialize() override
    {
        auto* sampler = new ProsacSampler(this->sac_params_.rng,
                                          this->estimator_.SampleSize());
        return Base::Initialize(sampler);
    }
};

} // namespace tl
