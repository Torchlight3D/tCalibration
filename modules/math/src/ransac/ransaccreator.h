#pragma once

#include <tMath/Types>

#include "exhaustiveransac.h"
#include "lmed.h"
#include "prosac.h"
#include "ransac.h"

namespace tl {

// TODO: How to use template factory
template <class Estimator>
std::unique_ptr<SampleConsensus<Estimator>> createRansac(
    RansacType type, const SacParameters& params, const Estimator& estimator)
{
    std::unique_ptr<SampleConsensus<Estimator>> sac;
    switch (type) {
        case RansacType::RANSAC:
            sac.reset(new RANSAC<Estimator>(params, estimator));
            break;
        case RansacType::PROSAC:
            sac.reset(new Prosac<Estimator>(params, estimator));
            break;
        case RansacType::LMED:
            sac.reset(new LMed<Estimator>(params, estimator));
            break;
        case RansacType::EXHAUSTIVE:
            sac.reset(new ExhaustiveRansac<Estimator>(params, estimator));
            break;
        default:
            sac.reset(new RANSAC<Estimator>(params, estimator));
            break;
    }

    CHECK(sac->Initialize()) << "Failed to initialize ransac estimator";

    return sac;
}

} // namespace tl
