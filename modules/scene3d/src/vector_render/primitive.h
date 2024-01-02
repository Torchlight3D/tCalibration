#pragma once

#include <vector>

#ifdef WIN32
#include <windows.h>
#endif

#include <GL/gl.h>

#include "axis_aligned_box.h"
#include "nvector3.h"
#include "types.h"
#include "vector3.h"

namespace vrender {

class Feedback3DColor;
class Primitive;

#define EPS_SMOOTH_LINE_FACTOR 0.06 /* Lower for better smooth lines. */

//  A Feedback3DColor is a structure containing informations about a vertex
//  projected into the frame buffer.

class Feedback3DColor
{
public:
    Feedback3DColor(GLfloat *loc)
        : _pos(loc[0], loc[1], loc[2]),
          _red(loc[3]),
          _green(loc[4]),
          _blue(loc[5]),
          _alpha(loc[6])
    {
    }

    inline FLOAT x() const { return _pos[0]; }
    inline FLOAT y() const { return _pos[1]; }
    inline FLOAT z() const { return _pos[2]; }
    inline GLfloat red() const { return _red; }
    inline GLfloat green() const { return _green; }
    inline GLfloat blue() const { return _blue; }
    inline GLfloat alpha() const { return _alpha; }
    inline const Vector3 &pos() const { return _pos; }

    inline Feedback3DColor operator+(const Feedback3DColor &v) const
    {
        return Feedback3DColor(x() + v.x(), y() + v.y(), z() + v.z(),
                               red() + v.red(), green() + v.green(),
                               blue() + v.blue(), alpha() + v.alpha());
    }
    inline Feedback3DColor operator*(const GLFLOAT &f) const
    {
        return Feedback3DColor(x() * f, y() * f, z() * f, red() * GLfloat(f),
                               green() * GLfloat(f), blue() * GLfloat(f),
                               alpha() * GLfloat(f));
    }
    friend inline Feedback3DColor operator*(const GLFLOAT &f,
                                            const Feedback3DColor &F)
    {
        return F * f;
    }

    static size_t sizeInBuffer() { return 7; }

protected:
    Feedback3DColor(FLOAT x, FLOAT y, FLOAT z, GLfloat r, GLfloat g, GLfloat b,
                    GLfloat a)
        : _pos(x, y, z), _red(r), _green(g), _blue(b), _alpha(a)
    {
    }

    Vector3 _pos;
    GLfloat _red;
    GLfloat _green;
    GLfloat _blue;
    GLfloat _alpha;
};

class Primitive
{
public:
    Primitive() {}
    virtual ~Primitive() {}

    virtual const Feedback3DColor &sommet3DColor(size_t) const = 0;

    // Renvoie le ieme vertex modulo le nombre de vertex.
    virtual const Vector3 &vertex(size_t) const = 0;
#ifdef A_FAIRE
    virtual FLOAT Get_I_EPS(Primitive *) const;
    Vect3 VerticalProjectPointOnSupportPlane(const Vector3 &) const;
    void IntersectPrimitiveWithSupportPlane(Primitive *, int[], FLOAT[],
                                            Vect3 *&, Vect3 *&);
    inline FLOAT Equation(const Vect3 &p) { return p * _normal - _C; }
    virtual void Split(Vect3, FLOAT, Primitive *&, Primitive *&) = 0;
    void GetSigns(Primitive *, int *&, FLOAT *&, int &, int &, FLOAT);
    FLOAT Const() const { return _C; }

    int depth() const { return _depth; }
    void setDepth(int d) const { _depth = d; }
#endif
    virtual AxisAlignedBox3D bbox() const = 0;
    virtual size_t nbVertices() const = 0;

protected:
    int _vibility;
};

class Point : public Primitive
{
public:
    Point(const Feedback3DColor &f);
    virtual ~Point() {}

    const Vector3 &vertex(size_t) const override;
    size_t nbVertices() const override { return 1; }
    const Feedback3DColor &sommet3DColor(size_t) const override;
    AxisAlignedBox3D bbox() const override;

private:
    Feedback3DColor _position_and_color;
};

class Segment : public Primitive
{
public:
    Segment(const Feedback3DColor &p1, const Feedback3DColor &p2)
        : P1(p1), P2(p2)
    {
    }
    virtual ~Segment() {}

    size_t nbVertices() const override { return 2; }
    const Vector3 &vertex(size_t) const override;
    const Feedback3DColor &sommet3DColor(size_t i) const override;
    AxisAlignedBox3D bbox() const override;
#ifdef A_FAIRE
    virtual void Split(const Vector3 &, FLOAT, Primitive *&, Primitive *&);
#endif

protected:
    Feedback3DColor P1;
    Feedback3DColor P2;
};

class Polygone : public Primitive
{
public:
    Polygone(const std::vector<Feedback3DColor> &);
    virtual ~Polygone() {}

#ifdef A_FAIRE
    virtual int IsAPolygon() { return 1; }
    virtual void Split(const Vector3 &, FLOAT, Primitive *&, Primitive *&);
    void InitEquation(double &, double &, double &, double &);
#endif

    const Feedback3DColor &sommet3DColor(size_t) const override;
    const Vector3 &vertex(size_t) const override;
    size_t nbVertices() const override { return _vertices.size(); }
    AxisAlignedBox3D bbox() const override;

    double equation(const Vector3 &p) const;
    const NVector3 &normal() const { return _normal; }
    double c() const { return _c; }

    FLOAT FlatFactor() const { return anglefactor; }

protected:
    virtual void initNormal();

    void CheckInfoForPositionOperators();

    AxisAlignedBox3D _bbox;
    std::vector<Feedback3DColor> _vertices;
    // std::vector<FLOAT> _sommetsProjetes ;
    // Vector3 N,M,L ;
    double anglefactor; //  Determine a quel point un polygone est plat.
    // Comparer a FLAT_POLYGON_EPS
    double _c;
    NVector3 _normal;
};

std::ostream &operator<<(std::ostream &, const Feedback3DColor &);

} // namespace vrender
