#pragma once

#include <Eigen/Core>

#include "vignetting.h"

namespace tl {

//  Dense vignetting model, with one attenuation factor specified per pixel. The
//  1D parameter vector, representing the 2D matrix of attenuation factors, is
//  organized in row-major order.
class DenseVignetting final : public Vignetting_<DenseVignetting>
{
public:
    using Vignetting_::Vignetting_;

    template <typename T>
    inline static T evaluate(const T* params, int width, int height, double u,
                             double v)
    {
        // initialize attenuation

        T result = T(0);

        // force point inside image

        u = std::max(0.5, std::min(width - 0.5, u));
        v = std::max(0.5, std::min(height - 0.5, v));

        // compute values and weigths for bilinear interpolation

        const int x0 = u - 0.5;
        const int y0 = v - 0.5;
        const T wx1 = T(u - x0 - 0.5);
        const T wy1 = T(v - y0 - 0.5);
        const T wx0 = T(1) - wx1;
        const T wy0 = T(1) - wy1;

        // perform final bilinear interpolation of all attenuation factors

        result += wy0 * wx0 * params[(y0 + 0) * width + (x0 + 0)];
        result += wy0 * wx1 * params[(y0 + 0) * width + (x0 + 1)];
        result += wy1 * wx0 * params[(y0 + 1) * width + (x0 + 0)];
        result += wy1 * wx1 * params[(y0 + 1) * width + (x0 + 1)];

        return result;
    }

    inline static void resetParameters(double* params, int width, int height)
    {
        const auto count = getNumParameters(width, height);
        Eigen::Map<Eigen::VectorXd>{params, static_cast<Eigen::Index>(count)}.setOnes();
    }

    inline static size_t getNumParameters(int width, int height)
    {
        return width * height;
    }
};

} // namespace tl
