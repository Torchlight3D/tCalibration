#pragma once

namespace vrender {

class Vector2;
class Vector3;

template <class T>
class AxisAlignedBox
{
public:
    inline AxisAlignedBox() : AxisAlignedBox(T::inf, -T::inf) {}
    inline AxisAlignedBox(const T& v) : AxisAlignedBox(v, v) {}
    AxisAlignedBox(const T& v, const T& w);

    const T& mini() const { return _min; }
    const T& maxi() const { return _max; }

    void include(const T& v);
    void include(const AxisAlignedBox<T>& box);

private:
    T _min;
    T _max;
};

using AxisAlignedBox2D = AxisAlignedBox<Vector2>;
using AxisAlignedBox3D = AxisAlignedBox<Vector3>;

///----------------------- Implementation --------------------------
///

template <class T>
AxisAlignedBox<T>::AxisAlignedBox(const T& v, const T& w) : _min(v), _max(v)
{
    include(w);
}

template <class T>
void AxisAlignedBox<T>::include(const T& v)
{
    _min = T::mini(_min, v);
    _max = T::maxi(_max, v);
}

template <class T>
void AxisAlignedBox<T>::include(const AxisAlignedBox<T>& b)
{
    include(b._min);
    include(b._max);
}

} // namespace vrender
