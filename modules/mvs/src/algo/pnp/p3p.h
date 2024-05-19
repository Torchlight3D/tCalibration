#pragma once

#include <tMath/Eigen/Types>

namespace tl {

bool P3P(const Vector2dList& imagePoints, const Vector3dList& worldPoints,
         Matrix3dList& rotations, Vector3dList& translations);

} // namespace tl
