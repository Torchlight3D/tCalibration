#pragma once

namespace tl {

// A wrapper around the std random generator with default engine mt19937
class RandomNumberGenerator
{
public:
    RandomNumberGenerator();
    explicit RandomNumberGenerator(unsigned int seed);

    void Seed(unsigned int seed) const;

    double RandDouble(double lower, double upper) const;
    inline auto Rand(double lower, double upper) const
    {
        return RandDouble(lower, upper);
    }

    float RandFloat(float lower, float upper) const;
    inline auto Rand(float lower, float upper) const
    {
        return RandFloat(lower, upper);
    }

    int RandInt(int lower, int upper) const;
    inline auto Rand(int lower, int upper) const
    {
        return RandInt(lower, upper);
    }

    double RandGaussian(double mean, double stddev) const;
};

} // namespace tl
