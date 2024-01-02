#pragma once

#include <cmath>
#include <iostream>

#include <QDomElement>

namespace viewer {

class Vec
{
public:
    union
    {
        struct
        {
            qreal x, y, z;
        };
        qreal v_[3];
    };

    Vec() : Vec(0.0, 0.0, 0.0) {}
    template <class C>
    explicit Vec(const C &c) : Vec(c[0], c[1], c[2])
    {
    }
    explicit Vec(const QDomElement &element);
    Vec(qreal X, qreal Y, qreal Z) : x(X), y(Y), z(Z) {}

    void setValue(qreal X, qreal Y, qreal Z)
    {
        x = X;
        y = Y;
        z = Z;
    }

    qreal operator[](int i) const { return v_[i]; }
    qreal &operator[](int i) { return v_[i]; }

    operator const qreal *() const { return v_; }
    operator qreal *() { return v_; }

    operator const float *() const
    {
        static float *const result = new float[3];
        result[0] = (float)x;
        result[1] = (float)y;
        result[2] = (float)z;
        return result;
    }

    friend Vec operator+(const Vec &a, const Vec &b)
    {
        return {a.x + b.x, a.y + b.y, a.z + b.z};
    }
    friend Vec operator-(const Vec &a, const Vec &b)
    {
        return {a.x - b.x, a.y - b.y, a.z - b.z};
    }

    friend Vec operator-(const Vec &a) { return {-a.x, -a.y, -a.z}; }
    friend Vec operator*(const Vec &a, qreal k)
    {
        return {a.x * k, a.y * k, a.z * k};
    }

    /*! Returns the product of a scalar with the vector. */
    friend Vec operator*(qreal k, const Vec &a) { return a * k; }

    friend Vec operator/(const Vec &a, qreal k)
    {
#ifndef QT_NO_DEBUG
        if (fabs(k) < 1.0e-10)
            qWarning("Vec::operator / : dividing by a null value (%f)", k);
#endif
        return Vec(a.x / k, a.y / k, a.z / k);
    }

    friend bool operator!=(const Vec &a, const Vec &b) { return !(a == b); }
    friend bool operator==(const Vec &a, const Vec &b)
    {
        constexpr qreal epsilon{1.0e-10};
        return (a - b).squaredNorm() < epsilon;
    }

    Vec &operator+=(const Vec &a)
    {
        x += a.x;
        y += a.y;
        z += a.z;
        return *this;
    }

    Vec &operator-=(const Vec &a)
    {
        x -= a.x;
        y -= a.y;
        z -= a.z;
        return *this;
    }

    Vec &operator*=(qreal k)
    {
        x *= k;
        y *= k;
        z *= k;
        return *this;
    }

    Vec &operator/=(qreal k)
    {
#ifndef QT_NO_DEBUG
        if (fabs(k) < 1.0e-10)
            qWarning("Vec::operator /= : dividing by a null value (%f)", k);
#endif
        x /= k;
        y /= k;
        z /= k;
        return *this;
    }

    friend qreal operator*(const Vec &a, const Vec &b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    friend Vec operator^(const Vec &a, const Vec &b) { return cross(a, b); }

    friend Vec cross(const Vec &a, const Vec &b)
    {
        return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
    }

    Vec orthogonalVec() const;

    qreal squaredNorm() const { return x * x + y * y + z * z; }
    qreal norm() const { return sqrt(squaredNorm()); }

    qreal normalize()
    {
        const qreal n = norm();
#ifndef QT_NO_DEBUG
        if (n < 1.0e-10)
            qWarning("Vec::normalize: normalizing a null vector (norm=%f)", n);
#endif
        *this /= n;
        return n;
    }

    Vec unit() const
    {
        Vec v = *this;
        v.normalize();
        return v;
    }

    void projectOnAxis(const Vec &direction);
    void projectOnPlane(const Vec &normal);

    QDomElement domElement(const QString &name, QDomDocument &document) const;
    void initFromDOMElement(const QDomElement &element);

#ifdef DOXYGEN
    std::ostream &operator<<(std::ostream &o, const viewer::Vec &);
#endif
};

} // namespace viewer

std::ostream &operator<<(std::ostream &o, const viewer::Vec &vec);
