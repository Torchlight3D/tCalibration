#pragma once

#include <Eigen/Core>

#include "cameraresponse.h"

namespace tl {

// 3rd order polynomial inverse-response model.
class Poly3Response final : public CameraResponse_<Poly3Response, 3>
{
public:
    using CameraResponse_::CameraResponse_;

    template <typename T>
    inline static T evaluate(const T* params, T value)
    {
        // initialize attenuation
        T pow = value;
        T result = T(0);

        // add each term of the polynomial
        for (size_t i{0}; i < kNumParameters; ++i) {
            result += pow * params[i];
            pow *= value;
        }

        return result;
    }

    template <typename T>
    inline static void resetParameters(T* params)
    {
        Eigen::Map<Eigen::Vector3<T>>{params} =
            Eigen::Vector3<T>{T(1), T(0), T(0)};
    }
};

// 4th order polynomial inverse-response model.
class Poly4Response final : public CameraResponse_<Poly4Response, 4>
{
public:
    using CameraResponse_::CameraResponse_;

    template <typename T>
    static inline T evaluate(const T* params, T value)
    {
        // initialize attenuation
        T pow = value;
        T result = T(0);

        // add each term of the polynomial
        for (size_t i{0}; i < kNumParameters; ++i) {
            result += pow * params[i];
            pow *= value;
        }

        return result;
    }

    template <typename T>
    static inline void resetParameters(T* params)
    {
        Eigen::Map<Eigen::Vector4<T>>{params} =
            Eigen::Vector4<T>{T(1), T(0), T(0), T(0)};
    }
};

} // namespace tl
