#include "GLine2D.h"

#include "MathUtil.h"

namespace AprilTags {

GLine2D::GLine2D() : didNormalizeSlope(false), didNormalizeP(false) {}

GLine2D::GLine2D(float slope, float b)
    : delta(1.f, slope), p(0, b), didNormalizeSlope(false), didNormalizeP(false)
{
}

GLine2D::GLine2D(float dX, float dY, const cv::Point2f& pt)
    : delta(dX, dY), p(pt), didNormalizeSlope(false), didNormalizeP(false)
{
}

GLine2D::GLine2D(const cv::Point2f& p1, const cv::Point2f& p2)
    : delta(p2 - p1), p(p1), didNormalizeSlope(false), didNormalizeP(false)
{
}

float GLine2D::getLineCoordinate(const cv::Point2f& pt)
{
    normalizeSlope();
    return pt.dot(delta);
}

cv::Point2f GLine2D::getPointOfCoordinate(float coord)
{
    normalizeP();
    return p + coord * delta;
}

cv::Point2f GLine2D::intersectionWith(const GLine2D& line) const
{
    float m00 = delta.x;
    float m01 = -line.getDx();
    float m10 = delta.y;
    float m11 = -line.getDy();

    // determinant of 'm'
    float det = m00 * m11 - m01 * m10;

    // parallel lines? if so, return (-1,0).
    if (std::abs(det) < 1e-10) {
        return {-1.f, 0.f};
    }

    // inverse of 'm'
    float i00 = m11 / det;
    // float i11 = m00/det;
    float i01 = -m01 / det;
    // float i10 = -m10/det;

    float b00 = line.getFirst() - p.x;
    float b10 = line.getSecond() - p.y;

    float x00 = i00 * b00 + i01 * b10;

    return delta * x00 + p;
}

GLine2D GLine2D::lsqFitXYW(const std::vector<XYWeight>& xyweights)
{
    float Cxx = 0, Cyy = 0, Cxy = 0, Ex = 0, Ey = 0, mXX = 0, mYY = 0, mXY = 0,
          mX = 0, mY = 0;
    float n = 0;

    int idx = 0;
    for (unsigned int i = 0; i < xyweights.size(); i++) {
        float x = xyweights[i].pos.x;
        float y = xyweights[i].pos.y;
        float alpha = xyweights[i].weight;

        mY += y * alpha;
        mX += x * alpha;
        mYY += y * y * alpha;
        mXX += x * x * alpha;
        mXY += x * y * alpha;
        n += alpha;

        idx++;
    }

    Ex = mX / n;
    Ey = mY / n;
    Cxx = mXX / n - MathUtil::square(mX / n);
    Cyy = mYY / n - MathUtil::square(mY / n);
    Cxy = mXY / n - (mX / n) * (mY / n);

    // find dominant direction via SVD
    float phi = 0.5f * std::atan2(-2 * Cxy, (Cyy - Cxx));

    // why is this needed if he never uses it?
    //  float rho = Ex*cos(phi) + Ey*sin(phi);

    // compute line parameters
    return {-std::sin(phi), std::cos(phi), cv::Point2f{Ex, Ey}};
}

void GLine2D::normalizeSlope()
{
    if (!didNormalizeSlope) {
        float mag = cv::norm(delta);
        delta /= mag;
        didNormalizeSlope = true;
    }
}

void GLine2D::normalizeP()
{
    if (!didNormalizeP) {
        normalizeSlope();
        // we already have a point (P) on the line, and we know the line vector
        // U and its perpendicular vector V: so, P'=P.*V *V
        float dotprod = delta.cross(p);
        p = {-delta.y * dotprod, delta.x * dotprod};
        didNormalizeP = true;
    }
}

} // namespace AprilTags
