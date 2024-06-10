#pragma once

#include "vignetting.h"

namespace tl {

// Uniform vignetting model, used in the trivial case where a camera has
// effectively no attenuation
class UniformVignetting final : public Vignetting_<UniformVignetting>
{
public:
    using Vignetting_::Vignetting_;

    template <typename T>
    inline static T evaluate(const T * /*params*/, int /*width*/,
                             int /*height*/, double /*u*/, double /*v*/)
    {
        return T(1);
    }

    // Do nothing
    inline static void resetParameters(double * /*params*/, int /*width*/,
                                       int /*height*/)
    {
    }

    // No parameters
    inline static size_t getNumParameters(int /*width*/, int /*height*/)
    {
        return 0;
    }
};

} // namespace tl
