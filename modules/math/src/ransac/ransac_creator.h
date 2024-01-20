#pragma once

#include "exhaustive_ransac.h"
#include "lmed.h"
#include "prosac.h"
#include "ransac.h"

namespace tl {

enum class RansacType
{
    RANSAC = 0,
    PROSAC = 1,
    LMED = 2,
    EXHAUSTIVE = 3
};

template <class Estimator>
std::unique_ptr<SampleConsensusEstimator<Estimator>> createRansac(
    RansacType type, const SacParameters& params, const Estimator& estimator)
{
    std::unique_ptr<SampleConsensusEstimator<Estimator>> sac;
    switch (type) {
        case RansacType::RANSAC:
            sac.reset(new Ransac<Estimator>(params, estimator));
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
            sac.reset(new Ransac<Estimator>(params, estimator));
            break;
    }

    CHECK(sac->Initialize()) << "Failed to initialize ransac estimator";
    return sac;
}

} // namespace tl
