#include "line_2d.h"

#include <tMath/MathBase>

using namespace tl;

namespace apriltags {

GLine2D::GLine2D() : GLine2D(0.f, 0.f, cv::Point2f{0.f, 0.f}) {}

GLine2D::GLine2D(float slope, float b)
    : GLine2D(1.f, slope, cv::Point2f{0.f, b})
{
}

GLine2D::GLine2D(const cv::Point2f& p1, const cv::Point2f& p2)
    : GLine2D(p2.x - p1.x, p2.y - p1.y, p1)
{
}

GLine2D::GLine2D(float dx, float dy, const cv::Point2f& pt)
    : _dx(dx), _dy(dy), _pt(pt)
{
    // normalize slope
    const float mag = std::sqrt(_dx * _dx + _dy * _dy);
    _dx /= mag;
    _dy /= mag;

    // normalized point
    // we already have a point (P) on the line, and we know the line vector
    // U and its perpendicular vector V: so, P'=P.*V *V
    float dotprod = -_dy * _pt.x + _dx * _pt.y;
    _pt = {-_dy * dotprod, _dx * dotprod};
}

float GLine2D::lineCoordinate(const cv::Point2f& pt) const
{
    return pt.x * _dx + pt.y * _dy;
}

cv::Point2f GLine2D::pointOfCoordinate(float coord) const
{
    return {_pt.x + coord * _dx, _pt.y + coord * _dy};
}

cv::Point2f GLine2D::intersectWith(const GLine2D& line) const
{
    float m00 = _dx;
    float m01 = -line.dx();
    float m10 = _dy;
    float m11 = -line.dy();

    // determinant of 'm'
    float det = m00 * m11 - m01 * m10;

    // parallel lines? if so, return (-1,0).
    if (fabs(det) < 1e-10) {
        return {-1, 0};
    }

    // inverse of 'm'
    float i00 = m11 / det;
    // float i11 = m00/det;
    float i01 = -m01 / det;
    // float i10 = -m10/det;

    float b00 = line.getFirst() - _pt.x;
    float b10 = line.getSecond() - _pt.y;

    float x00 = i00 * b00 + i01 * b10;

    return {_dx * x00 + _pt.x, _dy * x00 + _pt.y};
}

GLine2D GLine2D::fitLine(const std::vector<WeightedPointF>& weightedPoints)
{
    float mXX{0.f}, mYY{0.f}, mXY{0.f}, mX{0.f}, mY{0.f};
    float n{0.f};

    int idx = 0;
    for (const auto& weightedPoint : weightedPoints) {
        const float& x = weightedPoint.point.x;
        const float& y = weightedPoint.point.y;
        const float& alpha = weightedPoint.weight;

        mY += y * alpha;
        mX += x * alpha;
        mYY += y * y * alpha;
        mXX += x * x * alpha;
        mXY += x * y * alpha;
        n += alpha;

        idx++;
    }

    const float Ex = mX / n;
    const float Ey = mY / n;
    const float Cxx = mXX / n - math::square(mX / n);
    const float Cyy = mYY / n - math::square(mY / n);
    const float Cxy = mXY / n - (mX / n) * (mY / n);

    // find dominant direction via SVD
    float phi = 0.5f * std::atan2(-2 * Cxy, (Cyy - Cxx));
    // float rho = Ex*cos(phi) + Ey*sin(phi); //why is this needed if he never
    // uses it?

    // compute line parameters
    return {-std::sin(phi), std::cos(phi), cv::Point2f{Ex, Ey}};
}

} // namespace apriltags
