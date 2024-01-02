#pragma once

#ifdef WIN32
#include <windows.h>
#endif

#include <GL/gl.h>

namespace vrender {

using FLOAT = double;
using GLFLOAT = GLdouble;

#ifdef A_VOIR
using DVector3 = T_Vect3<double>;
using Vector2 = T_Vect2<double>;
#endif

class Primitive;
using PrimitivePtr = Primitive *;

constexpr float FLAT_POLYGON_EPS = 1e-5f;

} // namespace vrender
