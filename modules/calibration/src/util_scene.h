#pragma once

#include <AxMVS/Scene>

namespace thoht {

class CalibBoardBase;

namespace mvs {

void setupBoardInScene(Scene::Ptr scene, const CalibBoardBase& board);

std::string makeUniqueViewName(CameraId id, double timestamp);

} // namespace mvs

} // namespace thoht
