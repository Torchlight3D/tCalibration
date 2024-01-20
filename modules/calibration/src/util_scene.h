#pragma once

#include <tMvs/Scene>

namespace tl {

class CalibBoardBase;

namespace mvs {

void setupBoardInScene(Scene::Ptr scene, const CalibBoardBase& board);

std::string makeUniqueViewName(CameraId id, double timestamp);

} // namespace mvs

} // namespace tl
