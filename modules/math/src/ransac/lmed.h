#pragma once

#include "lmed_quality_measurement.h"
#include "random_sampler.h"
#include "sampleconsensus.h"

namespace tl {

template <class ModelEstimator>
class LMed : public SampleConsensusEstimator<ModelEstimator>
{
    using Base = SampleConsensusEstimator<ModelEstimator>;

public:
    using Data = typename ModelEstimator::Data;
    using Model = typename ModelEstimator::Model;

    using Base::Base;

    bool Initialize() override
    {
        const bool initialized = Base::Initialize(new RandomSampler(
            this->sac_params_.rng, this->estimator_.SampleSize()));
        this->quality_measurement_.reset(
            new LmedQualityMeasurement(this->estimator_.SampleSize()));
        return initialized;
    }
};

} // namespace tl
