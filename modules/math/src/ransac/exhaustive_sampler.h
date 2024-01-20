﻿#pragma once

#include "sampler.h"

namespace tl {

// This class exhaustively generates all possible combinations for the data
// input. We limit this sampler to only enumerate combinations with sample sizes
// of 2 for simplicity.
class ExhaustiveSampler : public Sampler
{
public:
    ExhaustiveSampler(const std::shared_ptr<RandomNumberGenerator>& rng,
                      int min_num_samples);

    bool Initialize(int num_datapoints) override;

    // Samples the input variable data and fills the vector subset with the
    // random samples.
    bool Sample(std::vector<int>* subset) override;

private:
    int num_datapoints_;
    // We generate combinations by essentially iterating over the nested loops:
    //   for (int i = 0; i < num_datapoints; i++) {
    //     for (int j = i + 1; j < num_datapoints; j++) {
    //       sample = (i, j);
    //     }
    //   }
    //
    // We update i and j accordingly so that we do not need to store any other
    // data.
    int i_, j_;
};

} // namespace tl
