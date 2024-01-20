#pragma once

namespace tl {

// WARNING: a placeholder, not ready yet
class ImuNoise
{
public:
    ImuNoise() : ImuNoise(0., 0.) {}
    ImuNoise(double noiseStd, double randomWalkStd);

    void setNoiseStddev(double stddev);
    double noise() const;

    void setRandomWalkStddev(double stddev);
    double randomWalk() const;

private:
    double noise_std_;
    double random_walk_std_;
};

} // namespace tl
