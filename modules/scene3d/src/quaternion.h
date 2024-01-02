#pragma once

#include <cmath>
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <QtGlobal>

#include "vec.h"

namespace viewer {

class Quaternion
{
    enum
    {
        Size = 4,
    };

public:
    Quaternion() : Quaternion(0., 0., 0., 1.) {}
    explicit Quaternion(const QDomElement &element);
    Quaternion(const Vec &axis, qreal angle) { setAxisAngle(axis, angle); }
    Quaternion(const Vec &from, const Vec &to);
    Quaternion(qreal q0, qreal q1, qreal q2, qreal w) : q{q0, q1, q2, w} {}

    Quaternion(const Quaternion &Q)
    {
        for (int i = 0; i < Size; ++i) {
            q[i] = Q.q[i];
        }
    }
    Quaternion &operator=(const Quaternion &Q)
    {
        for (int i = 0; i < 4; ++i) {
            q[i] = Q.q[i];
        }
        return (*this);
    }

    void setAxisAngle(const Vec &axis, qreal angle)
    {
        const qreal norm = axis.norm();
        // TODO: use isApprox0
        if (norm < 1e-8) {
            q[0] = 0.0;
            q[1] = 0.0;
            q[2] = 0.0;
            q[3] = 1.0;
        }
        else {
            const qreal sin_half_angle = sin(angle / 2.0);
            q[0] = sin_half_angle * axis[0] / norm;
            q[1] = sin_half_angle * axis[1] / norm;
            q[2] = sin_half_angle * axis[2] / norm;
            q[3] = cos(angle / 2.0);
        }
    }

    void setValue(qreal q0, qreal q1, qreal q2, qreal w)
    {
        q[0] = q0;
        q[1] = q1;
        q[2] = q2;
        q[3] = w;
    }

    void setFromRotationMatrix(const qreal m[3][3]);
    void setFromRotatedBasis(const Vec &X, const Vec &Y, const Vec &Z);

    Vec axis() const;
    qreal angle() const;
    void getAxisAngle(Vec &axis, qreal &angle) const;

    qreal operator[](int i) const { return q[i]; }
    qreal &operator[](int i) { return q[i]; }

    friend Quaternion operator*(const Quaternion &a, const Quaternion &b)
    {
        return {a.q[3] * b.q[0] + b.q[3] * a.q[0] + a.q[1] * b.q[2] -
                    a.q[2] * b.q[1],
                a.q[3] * b.q[1] + b.q[3] * a.q[1] + a.q[2] * b.q[0] -
                    a.q[0] * b.q[2],
                a.q[3] * b.q[2] + b.q[3] * a.q[2] + a.q[0] * b.q[1] -
                    a.q[1] * b.q[0],
                a.q[3] * b.q[3] - b.q[0] * a.q[0] - a.q[1] * b.q[1] -
                    a.q[2] * b.q[2]};
    }

    Quaternion &operator*=(const Quaternion &q)
    {
        *this = (*this) * q;
        return *this;
    }

    friend Vec operator*(const Quaternion &q, const Vec &v)
    {
        return q.rotate(v);
    }

    Vec rotate(const Vec &v) const;
    Vec inverseRotate(const Vec &v) const;

    Quaternion inverse() const { return {-q[0], -q[1], -q[2], q[3]}; }

    void invert()
    {
        q[0] = -q[0];
        q[1] = -q[1];
        q[2] = -q[2];
    }

    void negate()
    {
        invert();
        q[3] = -q[3];
    }

    qreal normalize()
    {
        const qreal norm =
            std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        for (int i = 0; i < Size; ++i) {
            q[i] /= norm;
        }
        return norm;
    }

    Quaternion normalized() const
    {
        qreal Q[4];
        const qreal norm =
            sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        for (int i = 0; i < 4; ++i) Q[i] = q[i] / norm;
        return Quaternion(Q[0], Q[1], Q[2], Q[3]);
    }

    const GLdouble *matrix() const;
    void getMatrix(GLdouble m[4][4]) const;
    void getMatrix(GLdouble m[16]) const;

    void getRotationMatrix(qreal m[3][3]) const;

    const GLdouble *inverseMatrix() const;
    void getInverseMatrix(GLdouble m[4][4]) const;
    void getInverseMatrix(GLdouble m[16]) const;

    void getInverseRotationMatrix(qreal m[3][3]) const;

    static Quaternion slerp(const Quaternion &a, const Quaternion &b, qreal t,
                            bool allowFlip = true);
    static Quaternion squad(const Quaternion &a, const Quaternion &tgA,
                            const Quaternion &tgB, const Quaternion &b,
                            qreal t);

    static qreal dot(const Quaternion &a, const Quaternion &b)
    {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    }

    Quaternion log();
    Quaternion exp();
    static Quaternion lnDif(const Quaternion &a, const Quaternion &b);
    static Quaternion squadTangent(const Quaternion &before,
                                   const Quaternion &center,
                                   const Quaternion &after);

    static Quaternion random();

    QDomElement domElement(const QString &name, QDomDocument &document) const;
    void initFromDOMElement(const QDomElement &element);

#ifdef DOXYGEN
    std::ostream &operator<<(std::ostream &o, const viewer::Vec &);
#endif

private:
    qreal q[Size]; // [x, y, z, w]
};

} // namespace viewer

std::ostream &operator<<(std::ostream &o, const viewer::Quaternion &);
