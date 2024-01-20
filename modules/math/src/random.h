#pragma once

#include <Eigen/Core>

namespace tl {

// A wrapper around the std random generator. Default engine mt19937
class RandomNumberGenerator
{
public:
    RandomNumberGenerator();
    explicit RandomNumberGenerator(unsigned int seed);

    void Seed(unsigned int seed) const;

    double RandDouble(double lower, double upper) const;
    inline double Rand(double lower, double upper) const
    {
        return RandDouble(lower, upper);
    }

    float RandFloat(float lower, float upper) const;
    inline float Rand(float lower, float upper) const
    {
        return RandFloat(lower, upper);
    }

    int RandInt(int lower, int upper) const;

    double RandGaussian(double mean, double stddev) const;

    // Return eigen types with random initialization.
    // Methods with no params random values between [-1, 1] as Eigen::Random().
    Eigen::Vector2d RandVector2d(double min, double max) const;
    inline auto RandVector2d() const { return RandVector2d(-1., 1.); }
    Eigen::Vector3d RandVector3d(double min, double max) const;
    inline auto RandVector3d() const { return RandVector3d(-1., 1.); }
    Eigen::Vector4d RandVector4d(double min, double max) const;
    inline auto RandVector4d() const { return RandVector4d(-1., 1.); }

    // Sets an Eigen type with random values between -1.0 and 1.0.
    // This is meant to replace the Eigen::Random() functionality.
    template <typename Derived>
    void SetRandom(Eigen::MatrixBase<Derived>* b) const
    {
        for (int r{0}; r < b->rows(); r++) {
            for (int c{0}; c < b->cols(); c++) {
                (*b)(r, c) = Rand(-1.0, 1.0);
            }
        }
    }
};

} // namespace tl
