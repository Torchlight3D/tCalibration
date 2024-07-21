#pragma once

#include "sampleconsensus.h"

namespace tl {

template <class Model>
class RANSAC final : public SampleConsensus<Model>
{
    using Base = SampleConsensus<Model>;

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
