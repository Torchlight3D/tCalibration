#pragma once

#include <concepts>
#include <optional>
#include <random>

namespace tl {

template <typename T>
concept ArithmeticType = std::is_arithmetic_v<T>;

// A wrapper around the std random generator. Default engine mt19937
class RandomNumberGenerator
{
public:
    RandomNumberGenerator();
    explicit RandomNumberGenerator(size_t seed);

    static RandomNumberGenerator& theRNG(
        const std::optional<size_t>& seed = {});

    void seed(size_t seed);

    template <std::integral T>
    inline T randInt(T min, T max)
    {
        std::uniform_int_distribution<T> distribution{min, max};
        return distribution(_rng);
    }

    template <std::signed_integral T>
    inline T randInt()
    {
        return randInt(T(-1), T(1));
    }

    template <std::floating_point T>
    inline T randFloat(T min, T max)
    {
        std::uniform_real_distribution<T> distribution{min, max};
        return distribution(_rng);
    }

    template <std::floating_point T>
    inline T randFloat()
    {
        return randFloat(T(-1), T(1));
    }

    template <ArithmeticType T>
    inline T rand(T min, T max)
    {
        if constexpr (std::signed_integral<T>) {
            return randInt(min, max);
        }

        if constexpr (std::floating_point<T>) {
            return randFloat(min, max);
        }

        // TODO: Return something here? But it's impossible to arrive here.
    }

    template <std::floating_point T>
    inline T randNorm(T mean, T sigma)
    {
        std::normal_distribution<T> distribution{mean, sigma};
        return distribution(_rng);
    }

private:
    std::mt19937_64 _rng;
};

} // namespace tl
