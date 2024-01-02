#include "imunoise.h"

namespace thoht {

ImuNoise::ImuNoise(double noiseStd, double randomWalkStd)
    : noise_std_(noiseStd), random_walk_std_(randomWalkStd)
{
}

void ImuNoise::setNoiseStddev(double stddev) { noise_std_ = stddev; }

double ImuNoise::noise() const { return noise_std_; }

void ImuNoise::setRandomWalkStddev(double stddev) { random_walk_std_ = stddev; }

double ImuNoise::randomWalk() const { return random_walk_std_; }

} // namespace thoht
