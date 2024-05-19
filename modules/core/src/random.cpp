#include "random.h"

#include <chrono>
#include <random>

namespace tl {

namespace {
std::mt19937 kRNG;
} // namespace

RandomNumberGenerator::RandomNumberGenerator()
{
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    kRNG.seed(seed);
}

RandomNumberGenerator::RandomNumberGenerator(unsigned seed) { kRNG.seed(seed); }

void RandomNumberGenerator::Seed(unsigned seed) const { kRNG.seed(seed); }

double RandomNumberGenerator::RandDouble(double lower, double upper) const
{
    std::uniform_real_distribution<double> distribution{lower, upper};
    return distribution(kRNG);
}

float RandomNumberGenerator::RandFloat(float lower, float upper) const
{
    std::uniform_real_distribution<float> distribution{lower, upper};
    return distribution(kRNG);
}

int RandomNumberGenerator::RandInt(int lower, int upper) const
{
    std::uniform_int_distribution<int> distribution{lower, upper};
    return distribution(kRNG);
}

double RandomNumberGenerator::RandGaussian(double mean, double stddev) const
{
    std::normal_distribution<double> distribution{mean, stddev};
    return distribution(kRNG);
}

} // namespace tl
