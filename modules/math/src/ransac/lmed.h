#pragma once

#include "sac_estimator.h"
#include "random_sampler.h"
#include "lmed_quality_measurement.h"

namespace thoht {

template <class ModelEstimator>
class LMed : public SampleConsensusEstimator<ModelEstimator>
{
    using Base = SampleConsensusEstimator<ModelEstimator>;

public:
    using Datum = typename ModelEstimator::Datum;
    using Model = typename ModelEstimator::Model;

    using Base::SampleConsensusEstimator;

    bool Initialize() override
    {
        const bool initialized = Base::Initialize(new RandomSampler(
            this->sac_params_.rng, this->estimator_.SampleSize()));
        this->quality_measurement_.reset(
            new LmedQualityMeasurement(this->estimator_.SampleSize()));
        return initialized;
    }
};

} // namespace thoht
