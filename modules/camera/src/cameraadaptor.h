#pragma once

#include "camodocal/Camera.h"
#include "simple/camera.h"

namespace tl {

Camera fromCocCamera(const camodocal::Camera::ConstPtr camera);

}
