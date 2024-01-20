#pragma once

#include "exhaustive_sampler.h"
#include "sac_estimator.h"

namespace tl {

template <class ModelEstimator>
class ExhaustiveRansac : public SampleConsensusEstimator<ModelEstimator>
{
public:
    using Datum = typename ModelEstimator::Datum;
    using Model = typename ModelEstimator::Model;

    using SampleConsensusEstimator<ModelEstimator>::SampleConsensusEstimator;

    bool Initialize()
    {
        auto* sampler = new ExhaustiveSampler(this->sac_params_.rng,
                                              this->estimator_.SampleSize());
        return SampleConsensusEstimator<ModelEstimator>::Initialize(sampler);
    }
};

} // namespace tl
