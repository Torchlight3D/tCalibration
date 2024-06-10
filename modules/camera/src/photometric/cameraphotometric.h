#pragma once

#include <memory>

#include "cameraresponse.h"
#include "vignetting.h"

namespace tl {

struct CameraPhotometric
{
    std::vector<std::shared_ptr<CameraResponse>> responses;
    std::vector<std::shared_ptr<Vignetting>> vignettings;
};

} // namespace tl
