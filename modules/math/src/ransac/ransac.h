#pragma once

#include "sampleconsensus.h"

namespace tl {

template <class ModelEstimator>
class RANSAC final : public SampleConsensus<ModelEstimator>
{
    using Base = SampleConsensus<ModelEstimator>;

public:
    using Base::Base;

    bool Initialize() override
    {
        auto* sampler =
            new RandomSampler(this->_estimator.SampleSize(), this->_params.rng);
        return Base::SetUpSampler(sampler);
    }
};

} // namespace tl
