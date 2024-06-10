#pragma once

#include "camodocal/Camera.h"
#include "geometric/camera.h"

namespace tl {

Camera fromCocCamera(const camodocal::Camera::ConstPtr camera);

}
