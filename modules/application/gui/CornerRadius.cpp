#include "CornerRadius.h"

namespace tl {

CornerRadius::CornerRadius(int radius)
    : CornerRadius(static_cast<double>(radius))
{
}

CornerRadius::CornerRadius(int left, int right)
    : CornerRadius(static_cast<double>(left), static_cast<double>(right))
{
}

CornerRadius::CornerRadius(int topLeft, int topRight, int bottomRight,
                           int bottomLeft)
    : CornerRadius(static_cast<double>(topLeft), static_cast<double>(topRight),
                   static_cast<double>(bottomRight),
                   static_cast<double>(bottomLeft))
{
}

CornerRadius::CornerRadius(double radius)
    : CornerRadius(radius, radius, radius, radius)
{
}

CornerRadius::CornerRadius(double left, double right)
    : CornerRadius(left, right, right, left)
{
}

CornerRadius::CornerRadius(double topLeft, double topRight, double bottomRight,
                           double bottomLeft)
    : topLeft(topLeft),
      topRight(topRight),
      bottomRight(bottomRight),
      bottomLeft(bottomLeft)
{
}

bool CornerRadius::hasSameRadius() const
{
    return topLeft == topRight && topLeft == bottomRight &&
           topLeft == bottomLeft;
}

bool CornerRadius::hasDifferentRadius() const { return !hasSameRadius(); }

QDebug operator<<(QDebug debug, const CornerRadius& radiuses)
{
    QDebugStateSaver saver(debug);
    debug.nospace() << "(" << radiuses.topLeft << ", " << radiuses.topRight
                    << ", " << radiuses.bottomRight << ", "
                    << radiuses.bottomLeft << ")";
    return debug;
}

CornerRadius& CornerRadius::operator=(double rhs)
{
    topLeft = rhs;
    topRight = rhs;
    bottomRight = rhs;
    bottomLeft = rhs;
    return *this;
}

CornerRadius CornerRadius::operator+(double rhs) const
{
    return {topLeft + rhs, topRight + rhs, bottomRight + rhs, bottomLeft + rhs};
}

CornerRadius CornerRadius::operator-(double rhs) const
{
    return {topLeft - rhs, topRight - rhs, bottomRight - rhs, bottomLeft - rhs};
}

bool CornerRadius::operator<(double rhs) const
{
    return topLeft < rhs && topRight < rhs && bottomRight < rhs &&
           bottomLeft < rhs;
}

bool CornerRadius::operator<=(double rhs) const
{
    return *this == rhs || *this < rhs;
}

bool CornerRadius::operator>(double rhs) const
{
    return topLeft > rhs && topRight > rhs && bottomRight > rhs &&
           bottomLeft > rhs;
}

bool CornerRadius::operator>=(double rhs) const
{
    return *this == rhs || *this > rhs;
}

bool CornerRadius::operator==(double rhs) const
{
    return topLeft == rhs && topRight == rhs && bottomRight == rhs &&
           bottomLeft == rhs;
}

bool CornerRadius::operator!=(double rhs) const
{
    return topLeft != rhs && topRight != rhs && bottomRight != rhs &&
           bottomLeft != rhs;
}

bool CornerRadius::operator==(const CornerRadius& o) const
{
    return topLeft == o.topLeft && topRight == o.topRight &&
           bottomRight == o.bottomRight && bottomLeft == o.bottomLeft;
}

bool CornerRadius::operator!=(const CornerRadius& rhs) const
{
    return !(*this == rhs);
}

} // namespace tl
