#pragma once

#include "qualitymeasurement.h"
#include "sampleconsensus.h"

namespace tl {

template <class Model>
class LMed final : public SampleConsensus<Model>
{
    using Base = SampleConsensus<Model>;

public:
    using Base::Base;

    bool Initialize() override
    {
        auto sampler =
            new RandomSampler(this->_estimator.SampleSize(), this->_params.rng);
        const bool initialized = Base::SetUpSampler(sampler);
        this->quality_measurement_.reset(
            new LmedQualityMeasurement(this->_estimator.SampleSize()));
        return initialized;
    }
};

} // namespace tl
