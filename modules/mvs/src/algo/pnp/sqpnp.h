#pragma once

#include <tMath/EigenTypes>

namespace tl {

bool SQPnP(const Vector2dList& imagePoints, const Vector3dList& worldPoints,
           QuaterniondList& rotations, Vector3dList& translations);

} // namespace tl
