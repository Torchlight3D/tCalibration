#pragma once

#include <vector>

#include "gpc.h"

namespace vrender {

class NVector3;
class Point;
class Polygone;
class Primitive;
class Segment;
class Vector2;
class Vector3;

class PrimitivePositioning
{
public:
    enum RelativePosition
    {
        Independent = 0x0,
        Upper = 0x1,
        Lower = 0x2
    };

    static int computeRelativePosition(const Primitive *p1,
                                       const Primitive *p2);

    static void splitPrimitive(Primitive *P, const NVector3 &v, double c,
                               Primitive *&prim_up, Primitive *&prim_lo);

    static void split(Segment *S, const NVector3 &v, double C,
                      Primitive *&P_plus, Primitive *&P_moins);
    static void split(Point *P, const NVector3 &v, double C, Primitive *&P_plus,
                      Primitive *&P_moins);
    static void split(Polygone *P, const NVector3 &v, double C,
                      Primitive *&P_plus, Primitive *&P_moins);

private:
    static void getsigns(const Primitive *P, const NVector3 &v, double C,
                         std::vector<int> &signs, std::vector<double> &zvals,
                         int &Smin, int &Smax, double I_EPS);

    static int computeRelativePosition(const Polygone *p1, const Polygone *p2);
    static int computeRelativePosition(const Polygone *p1, const Segment *p2);
    static int computeRelativePosition(const Polygone *p1, const Point *p2);
    static int computeRelativePosition(const Segment *p1, const Segment *p2);

    //  2D intersection/positioning methods. Parameter I_EPS may be positive of
    //  negative
    // depending on the wanted degree of conservativeness of the result.

    static bool pointOutOfPolygon_XY(const Vector3 &P, const Polygone *Q,
                                     double I_EPS);
    static bool intersectSegments_XY(const Vector2 &P1, const Vector2 &Q1,
                                     const Vector2 &P2, const Vector2 &Q2,
                                     double I_EPS, double &t1, double &t2);
    static gpc_polygon createGPCPolygon_XY(const Polygone *P);

    static int inverseRP(int);

    //  This value is *non negative*. It may be used with a negative sign
    // in 2D methods such as pointOutOfPolygon() so as to rule the behaviour of
    // the positionning.

    static double _EPS;
};

} // namespace vrender
