#pragma once

#include <ostream>
#include <Eigen/Geometry>

#include "util_eigen.h"
#include "ax_math_global.h"

namespace tl {

template <typename T>
class DualQuaternion;

template <typename T>
DualQuaternion<T> operator+(const DualQuaternion<T>& dq1,
                            const DualQuaternion<T>& dq2);
template <typename T>
DualQuaternion<T> operator-(const DualQuaternion<T>& dq1,
                            const DualQuaternion<T>& dq2);

template <typename T>
std::ostream& operator<<(std::ostream& out, const DualQuaternion<T>& dq);

template <typename T>
class AX_MATH_API DualQuaternion
{
public:
    using Quaternion_t = Eigen::Quaternion<T>;
    using Point_t = Eigen::Vector3<T>;
    using Vec_t = Eigen::Vector3<T>;

    DualQuaternion();
    DualQuaternion(const Quaternion_t& real, const Quaternion_t& dual);
    DualQuaternion(const Quaternion_t& real, const Vec_t& translation);

    void setFromScrew(T theta, T d, const Vec_t& l, const Vec_t& m);

    static DualQuaternion<T> identity();
    static DualQuaternion<T> zeros();
    static DualQuaternion<T> fromScrew(T theta, T d, const Vec_t& l,
                                       const Vec_t& m);

    Quaternion_t real() const;
    Quaternion_t dual() const;

    Vec_t translation() const;
    Quaternion_t translationQuaternion() const;
    inline Quaternion_t rotation() const { return real(); }

    DualQuaternion<T> conjugate() const;
    DualQuaternion<T> inverse() const;
    DualQuaternion<T> exp() const;
    DualQuaternion<T> log() const;

    void norm(T& real, T& dual) const;
    void normalize();
    DualQuaternion<T> normalized() const;

    Point_t transformPoint(const Point_t& point) const;
    Vec_t transformVector(const Vec_t& vector) const;

    Eigen::Matrix4<T> toMatrix() const;

    DualQuaternion<T> operator*(T scale) const;
    DualQuaternion<T> operator*(const DualQuaternion<T>& other) const;

    friend DualQuaternion<T> operator+
        <>(const DualQuaternion<T>& dq1, const DualQuaternion<T>& dq2);
    friend DualQuaternion<T> operator-
        <>(const DualQuaternion<T>& dq1, const DualQuaternion<T>& dq2);

    friend std::ostream& operator<< <>(std::ostream&, const DualQuaternion<T>&);

private:
    Quaternion_t _real;
    Quaternion_t _dual;
};

/////////////////////////// Implementation ////////////////////////////////
template <typename T>
DualQuaternion<T>::DualQuaternion()
    : _real(Quaternion_t::Identity()), _dual(0, 0, 0, 0)
{
}

template <typename T>
DualQuaternion<T>::DualQuaternion(const Quaternion_t& real,
                                  const Quaternion_t& dual)
    : _real(real), _dual(dual)
{
}

template <typename T>
DualQuaternion<T>::DualQuaternion(const Quaternion_t& r, const Vec_t& t)
    : _real(r.normalized()),
      _dual(T(0.5) * (Quaternion_t(T(0), t(0), t(1), t(2)) * _real).coeffs())
{
}

template <typename T>
void DualQuaternion<T>::setFromScrew(T theta, T d, const Vec_t& l,
                                     const Vec_t& m)
{
    _real = Eigen::AngleAxis<T>(theta, l);
    _dual.w() = -d / 2.0 * sin(theta / 2.0);
    _dual.vec() = sin(theta / 2.0) * m + d / 2.0 * cos(theta / 2.0) * l;
}

template <typename T>
DualQuaternion<T> DualQuaternion<T>::identity()
{
    return {Quaternion_t::Identity(), Quaternion_t(0, 0, 0, 0)};
}

template <typename T>
DualQuaternion<T> DualQuaternion<T>::zeros()
{
    return {Quaternion_t(T(0), T(0), T(0), T(0)),
            Quaternion_t(T(0), T(0), T(0), T(0))};
}

template <typename T>
DualQuaternion<T> DualQuaternion<T>::fromScrew(T theta, T d, const Vec_t& l,
                                               const Vec_t& m)
{
    DualQuaternion<T> res;
    res.setFromScrew(theta, d, l, m);
    return res;
}

template <typename T>
Eigen::Quaternion<T> DualQuaternion<T>::real() const
{
    return _real;
}

template <typename T>
Eigen::Quaternion<T> DualQuaternion<T>::dual() const
{
    return _dual;
}

template <typename T>
Eigen::Vector3<T> DualQuaternion<T>::translation() const
{
    Quaternion_t t(2.0 * (_dual * _real.conjugate()).coeffs());

    Vec_t tvec;
    tvec << t.x(), t.y(), t.z();

    return tvec;
}

template <typename T>
Eigen::Quaternion<T> DualQuaternion<T>::translationQuaternion() const
{
    Quaternion_t t(2.0 * (_dual * _real.conjugate()).coeffs());

    return t;
}

template <typename T>
DualQuaternion<T> DualQuaternion<T>::conjugate() const
{
    return {_real.conjugate(), _dual.conjugate()};
}

template <typename T>
DualQuaternion<T> DualQuaternion<T>::inverse() const
{
    T sqrLen0 = _real.squaredNorm();
    T sqrLenE = 2.0 * (_real.coeffs().dot(_dual.coeffs()));

    if (sqrLen0 > 0.0) {
        T invSqrLen0 = 1.0 / sqrLen0;
        T invSqrLenE = -sqrLenE / (sqrLen0 * sqrLen0);

        DualQuaternion<T> conj = conjugate();
        conj._real.coeffs() = invSqrLen0 * conj._real.coeffs();
        conj._dual.coeffs() =
            invSqrLen0 * conj._dual.coeffs() + invSqrLenE * conj._real.coeffs();

        return conj;
    }

    return DualQuaternion<T>::zeros();
}

template <typename T>
DualQuaternion<T> DualQuaternion<T>::exp() const
{
    const auto real = math::ExpQ(_real);
    const auto dual = real * _dual;

    return {real, dual};
}

template <typename T>
DualQuaternion<T> DualQuaternion<T>::log() const
{
    const auto real = math::LogQ(_real);
    auto dual = _real.conjugate() * _dual;
    const T scale = T(1) / _real.squaredNorm();
    dual.coeffs() *= scale;

    return {real, dual};
}

template <typename T>
void DualQuaternion<T>::norm(T& real, T& dual) const
{
    real = _real.norm();
    dual = _real.coeffs().dot(_dual.coeffs()) / real;
}

template <typename T>
void DualQuaternion<T>::normalize()
{
    T length = _real.norm();
    T lengthSqr = _real.squaredNorm();

    // real part is of unit length
    _real.coeffs() /= length;

    // real and dual parts are orthogonal
    _dual.coeffs() /= length;
    _dual.coeffs() -=
        (_real.coeffs().dot(_dual.coeffs()) * lengthSqr) * _real.coeffs();
}

template <typename T>
DualQuaternion<T> DualQuaternion<T>::normalized() const
{
    DualQuaternion<T> dq = *this;
    dq.normalize();

    return dq;
}

template <typename T>
Eigen::Vector3<T> DualQuaternion<T>::transformPoint(const Point_t& point) const
{
    DualQuaternion<T> dq =
        (*this) *
        DualQuaternion<T>(
            Quaternion_t::Identity(),
            Quaternion_t(0, point(0, 0), point(1, 0), point(2, 0))) *
        conjugate();

    Point_t pt(dq._dual.x(), dq._dual.y(), dq._dual.z());

    // translation
    pt += 2.0 * (_real.w() * _dual.vec() - _dual.w() * _real.vec() +
                 _real.vec().cross(_dual.vec()));

    return pt;
}

template <typename T>
Eigen::Vector3<T> DualQuaternion<T>::transformVector(const Vec_t& vector) const
{
    DualQuaternion<T> dq =
        (*this) *
        DualQuaternion<T>(
            Quaternion_t::Identity(),
            Quaternion_t(0, vector(0, 0), vector(1, 0), vector(2, 0))) *
        conjugate();

    return {dq._dual.x(), dq._dual.y(), dq._dual.z()};
}

template <typename T>
Eigen::Matrix4<T> DualQuaternion<T>::toMatrix() const
{
    Eigen::Matrix4<T> H = Eigen::Matrix4<T>::Identity();

    H.block(0, 0, 3, 3) = _real.toRotationMatrix();

    Quaternion_t t(2.0 * (_dual * _real.conjugate()).coeffs());
    H(0, 3) = t.x();
    H(1, 3) = t.y();
    H(2, 3) = t.z();

    return H;
}

template <typename T>
DualQuaternion<T> DualQuaternion<T>::operator*(T scale) const
{
    return {Quaternion_t(scale * _real.coeffs()),
            Quaternion_t(scale * _dual.coeffs())};
}

template <typename T>
DualQuaternion<T> DualQuaternion<T>::operator*(
    const DualQuaternion<T>& other) const
{
    return {_real * other._real, Quaternion_t((_real * other._dual).coeffs() +
                                              (_dual * other._real).coeffs())};
}

template <typename T>
DualQuaternion<T> operator+(const DualQuaternion<T>& dq1,
                            const DualQuaternion<T>& dq2)
{
    return {Quaternion_t(dq1._real.coeffs() + dq2._real.coeffs()),
            Quaternion_t(dq1._dual.coeffs() + dq2._dual.coeffs())};
}

template <typename T>
DualQuaternion<T> operator-(const DualQuaternion<T>& dq1,
                            const DualQuaternion<T>& dq2)
{
    return {Quaternion_t(dq1._real.coeffs() - dq2._real.coeffs()),
            Quaternion_t(dq1._dual.coeffs() - dq2._dual.coeffs())};
}

template <typename T>
DualQuaternion<T> operator*(T scale, const DualQuaternion<T>& dq)
{
    return {Quaternion_t(scale * dq.real().coeffs()),
            Quaternion_t(scale * dq.dual().coeffs())};
}

template <typename T>
std::ostream& operator<<(std::ostream& out, const DualQuaternion<T>& dq)
{
    out << dq._real.w() << " " << dq._real.x() << " " << dq._real.y() << " "
        << dq._real.z() << " " << dq._dual.w() << " " << dq._dual.x() << " "
        << dq._dual.y() << " " << dq._dual.z() << std::endl;
    return out;
}

template <typename T>
DualQuaternion<T> expDQ(
    const std::pair<Eigen::Quaternion<T>, Eigen::Quaternion<T>>& v8x1)
{
    const auto real = math::ExpQ(v8x1.first);
    const auto dual = real * v8x1.second;
    return {real, dual};
}

template <typename T>
std::pair<Eigen::Quaternion<T>, Eigen::Quaternion<T>> logDQ(
    const DualQuaternion<T>& dq)
{
    Eigen::Quaternion<T> real = math::LogQ(dq.real());
    Eigen::Quaternion<T> dual = dq.real().inverse() * dq.dual();
    return std::make_pair(real, dual);
}

using DualQuaternionf = DualQuaternion<float>;
using DualQuaterniond = DualQuaternion<double>;

} // namespace tl
