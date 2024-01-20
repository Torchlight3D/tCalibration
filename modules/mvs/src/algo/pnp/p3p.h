﻿#pragma once

#include <tMath/EigenTypes>

namespace tl {

bool P3P(const Vector2dList& imagePoints, const Vector3dList& worldPoints,
         Matrix3dList& rotations, Vector3dList& translations);

} // namespace tl
