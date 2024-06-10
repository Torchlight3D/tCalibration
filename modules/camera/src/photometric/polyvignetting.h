#pragma once

#include <Eigen/Core>

#include "vignetting.h"

namespace tl {

// Brief:
// 6th order even polynomial vignetting model.
//
// Ref:
// "Calibration and correction of vignetting effects with an application to 3D
// mapping.", by Alexandrov, S. V., etc. (IROS 2016)
class EvenPoly6Vignetting final : public Vignetting_<EvenPoly6Vignetting>
{
public:
    using Vignetting_::Vignetting_;

    template <typename T>
    inline static T evaluate(const T* params, int width, int height, double u,
                             double v)
    {
        // initialize attenuation
        T result = T(1);
        const Eigen::Vector2d point(u, v);
        const Eigen::Vector2d size(width, height);
        const Eigen::Vector2d center = 0.5 * size;
        const double radius = (point - center).norm();
        const double max_radius = center.norm();
        const double ratio = radius / max_radius;
        const T rr = ratio * ratio;
        T pow = rr;

        // add each term of the polynomial

        for (int i = 0; i < 3; ++i) {
            result += pow * params[i];
            pow *= rr;
        }

        return result;
    }

    inline static void resetParameters(double* params, int /*width*/,
                                       int /*height*/)
    {
        Eigen::Map<Eigen::Vector3d>{params}.setZero();
    }

    inline static size_t getNumParameters(int /*width*/, int /*height*/)
    {
        return 3;
    }
};

} // namespace tl
