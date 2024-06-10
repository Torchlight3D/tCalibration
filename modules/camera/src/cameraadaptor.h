#pragma once

#include "camodocal/Camera.h"
#include "geometric/camera.h"

namespace tl {

std::unique_ptr<Camera> fromCocCamera(const camodocal::Camera::ConstPtr camera);

}
