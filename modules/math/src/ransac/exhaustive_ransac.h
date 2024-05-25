#pragma once

#include "exhaustive_sampler.h"
#include "sampleconsensus.h"

namespace tl {

template <class ModelEstimator>
class ExhaustiveRansac : public SampleConsensusEstimator<ModelEstimator>
{
    using Base = SampleConsensusEstimator<ModelEstimator>;

public:
    using Data = typename ModelEstimator::Data;
    using Model = typename ModelEstimator::Model;

    using Base::Base;

    bool Initialize()
    {
        auto* sampler = new ExhaustiveSampler(this->sac_params_.rng,
                                              this->estimator_.SampleSize());
        return Base::Initialize(sampler);
    }
};

} // namespace tl
