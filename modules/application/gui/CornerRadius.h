
#pragma once

#include <QDebug>

namespace thoht {

struct CornerRadius
{
    double topLeft{0.}, topRight{0.}, bottomRight{0.}, bottomLeft{0.};

    CornerRadius() = default;
    CornerRadius(int radius);
    CornerRadius(int left, int right);
    CornerRadius(int topLeft, int topRight, int bottomRight, int bottomLeft);
    CornerRadius(double radius);
    CornerRadius(double left, double right);
    CornerRadius(double topLeft, double topRight, double bottomRight,
                 double bottomLeft);

    CornerRadius& operator=(double rhs);
    CornerRadius operator+(double rhs) const;
    CornerRadius operator-(double rhs) const;
    bool operator<(double rhs) const;
    bool operator<=(double rhs) const;
    bool operator>(double rhs) const;
    bool operator>=(double rhs) const;
    bool operator==(double rhs) const;
    bool operator!=(double rhs) const;
    bool operator==(const CornerRadius& other) const;
    bool operator!=(const CornerRadius& other) const;

    bool hasSameRadius() const;
    bool hasDifferentRadius() const;
};

QDebug operator<<(QDebug debug, const CornerRadius& radiuses);

} // namespace thoht

Q_DECLARE_METATYPE(thoht::CornerRadius);
