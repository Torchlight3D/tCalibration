#pragma once

#include <AxMath/EigenTypes>

namespace thoht {

bool SQPnP(const Vector2dList& imagePoints, const Vector3dList& worldPoints,
           QuaterniondList& rotations, Vector3dList& translations);

} // namespace thoht
