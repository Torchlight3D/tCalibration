#pragma once

#include <tMath/Eigen/Types>

namespace tl {

bool SQPnP(const Vector2dList& imagePoints, const Vector3dList& worldPoints,
           QuaterniondList& rotations, Vector3dList& translations);

} // namespace tl
