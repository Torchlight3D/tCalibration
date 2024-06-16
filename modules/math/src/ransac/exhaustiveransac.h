#pragma once

#include "sampleconsensus.h"

namespace tl {

// This class exhaustively generates all possible combinations for the data
// input. We limit this sampler to only enumerate combinations with sample sizes
// of 2 for simplicity.
class ExhaustiveSampler final : public Sampler
{
public:
    explicit ExhaustiveSampler(size_t minSampleSize,
                               std::shared_ptr<RandomNumberGenerator> rng = {});

    bool Initialize(size_t dataSize) override;

    bool Sample(std::vector<size_t>* indices) override;

private:
    size_t _dataSize;
    size_t _i, _j;
};

template <class ModelEstimator>
class ExhaustiveRansac final : public SampleConsensus<ModelEstimator>
{
    using Base = SampleConsensus<ModelEstimator>;

public:
    using Base::Base;

    bool Initialize() override
    {
        auto* sampler = new ExhaustiveSampler(this->_estimator.SampleSize(),
                                              this->_params.rng);
        return Base::SetUpSampler(sampler);
    }
};

} // namespace tl
