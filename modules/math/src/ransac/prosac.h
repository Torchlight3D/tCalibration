#pragma once

#include "sac_estimator.h"
#include "prosac_sampler.h"

namespace tl {

// Estimate a model using PROSAC. The Estimate method is inherited, but for
// PROSAC requires the data to be in sorted order by quality (with highest
// quality at index 0).
template <class ModelEstimator>
class Prosac : public SampleConsensusEstimator<ModelEstimator>
{
public:
    using Datum = typename ModelEstimator::Datum;
    using Model = typename ModelEstimator::Model;

    using SampleConsensusEstimator<ModelEstimator>::SampleConsensusEstimator;

    bool Initialize() override
    {
        auto* sampler = new ProsacSampler(this->sac_params_.rng,
                                          this->estimator_.SampleSize());
        return SampleConsensusEstimator<ModelEstimator>::Initialize(sampler);
    }
};

} // namespace tl
