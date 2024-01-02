#pragma once

#include "random_sampler.h"
#include "sac_estimator.h"

namespace thoht {

template <class ModelEstimator>
class Ransac : public SampleConsensusEstimator<ModelEstimator>
{
public:
    using Base = SampleConsensusEstimator<ModelEstimator>;
    using Datum = typename ModelEstimator::Datum;
    using Model = typename ModelEstimator::Model;

    using Base::SampleConsensusEstimator;

    bool Initialize()
    {
        auto* sampler = new RandomSampler(this->sac_params_.rng,
                                          this->estimator_.SampleSize());
        return Base::Initialize(sampler);
    }
};

} // namespace thoht
