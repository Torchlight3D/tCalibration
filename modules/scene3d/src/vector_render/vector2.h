#pragma once

#include <iostream>
#include <stdexcept>

namespace vrender {

class Vector3;

class Vector2
{
public:
    Vector2() : Vector2(0., 0.) {}
    Vector2(const Vector3& u);
    Vector2(double x, double y);
    Vector2(const Vector2& other);
    inline Vector2& operator=(const Vector2& other)
    {
        _xyz[0] = other._xyz[0];
        _xyz[1] = other._xyz[1];
        return *this;
    }
    ~Vector2();

    static const Vector2 inf;

    inline double x() const { return _xyz[0]; }
    inline double y() const { return _xyz[1]; }
    inline void setX(double x) { _xyz[0] = x; }
    inline void setY(double y) { _xyz[1] = y; }
    inline void setXY(double x, double y)
    {
        _xyz[0] = x;
        _xyz[1] = y;
    }

    friend bool operator==(const Vector2&, const Vector2&);
    friend bool operator!=(const Vector2&, const Vector2&);

    inline Vector2& operator+=(const Vector2& v)
    {
        _xyz[0] += v._xyz[0];
        _xyz[1] += v._xyz[1];
        return *this;
    }

    inline Vector2& operator-=(const Vector2& v)
    {
        _xyz[0] -= v._xyz[0];
        _xyz[1] -= v._xyz[1];
        return *this;
    }

    inline Vector2& operator*=(double f)
    {
        _xyz[0] *= f;
        _xyz[1] *= f;
        return *this;
    }
    inline Vector2& operator/=(double f)
    {
        _xyz[0] /= f;
        _xyz[1] /= f;
        return *this;
    }

    static Vector2 mini(const Vector2&, const Vector2&);
    static Vector2 maxi(const Vector2&, const Vector2&);

    inline Vector2 operator+(const Vector2& u) const
    {
        return Vector2(_xyz[0] + u._xyz[0], _xyz[1] + u._xyz[1]);
    }
    inline Vector2 operator-(const Vector2& u) const
    {
        return Vector2(_xyz[0] - u._xyz[0], _xyz[1] - u._xyz[1]);
    }

    inline double operator*(const Vector2& u) const
    {
        return _xyz[0] * u._xyz[0] + _xyz[1] * u._xyz[1];
    }

    inline double operator^(const Vector2& v) const
    {
        return _xyz[0] * v._xyz[1] - _xyz[1] * v._xyz[0];
    }

    Vector2 operator/(double v) { return Vector2(_xyz[0] / v, _xyz[1] / v); }
    Vector2 operator*(double v) { return Vector2(_xyz[0] * v, _xyz[1] * v); }

    double norm() const;
    double squareNorm() const;
    /// Should be used for most comparisons, for efficiency reasons.
    double infNorm() const;

    double operator[](int i) const
    {
        if ((i < 0) || (i > 1))
            throw std::runtime_error("Out of bounds in Vector2::operator[]");

        return _xyz[i];
    }

    double& operator[](int i)
    {
        if ((i < 0) || (i > 1))
            throw std::runtime_error("Out of bounds in Vector2::operator[]");

        return _xyz[i];
    }

private:
    double _xyz[2];
};

Vector2 operator*(double, const Vector2& vec2);
Vector2 operator-(const Vector2& vec2);

std::ostream& operator<<(std::ostream& os, const Vector2& vec2);

} // namespace vrender
