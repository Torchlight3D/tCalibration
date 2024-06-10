#pragma once

#include "cameraresponse.h"

namespace tl {

// Linear camera response model, used in the trivial case where the image
// intensities are always proportional to scene irradiance.
class LinearResponse final : public CameraResponse_<LinearResponse, 0>
{
public:
    using CameraResponse_::CameraResponse_;

    template <typename T>
    inline static T evaluate(const T * /*params*/, T value)
    {
        return value;
    }

    template <typename T>
    inline static void resetParameters(T * /*params*/)
    {
    }
};

} // namespace tl
