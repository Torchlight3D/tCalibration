#pragma once

#include <opencv2/opencv.hpp>

namespace orp {
namespace calibration {
//! Fits a grayscale model over an area of pixels.
/*! The model is of the form: c1*x + c2*y + c3*x*y + c4 = value
 *
 * We use this model to compute spatially-varying thresholds for
 * reading bits.
 */
template <typename Float>
class GrayModel
{
public:
    typedef cv::Matx<Float, 4, 4> Matrix;
    typedef cv::Vec<Float, 4> Vector;
    GrayModel() : AtA(), v(), Atb(), dirty(false) {}

    void add(Float x, Float y, Float z)
    {
        Float *a = AtA.val;
        Float xy = x * y;
        // compute A'A and A'*b incrementally
        a[0] += x * x;
        a[1] += x * y;
        a[2] += x * xy;
        a[3] += x;
        Atb[0] += x * z;
        a[5] += y * y;
        a[6] += y * xy;
        a[7] += y;
        Atb[1] += y * z;
        a[10] += xy * xy;
        a[11] += xy;
        Atb[2] += xy * z;
        a[15] += 1;
        Atb[3] += z;
        dirty = true;
    }

    Float interpolate(Float x, Float y)
    {
        if (dirty)
            update();
        return v[0] * x + v[1] * y + v[2] * x * y + v[3];
    }

private:
    void update()
    {
        Float *a = AtA.val;
        // do we have enough points?
        if (a[15] > 6) {
            // symmetrize AtA
            a[4] = a[1];
            a[8] = a[2];
            a[9] = a[6];
            a[12] = a[3];
            a[13] = a[7];
            a[14] = a[11];
            // pseudoinverse
            Matrix invAtA;
            cv::invert(AtA, invAtA, cv::DECOMP_SVD);
            // solution
            v = invAtA * Atb;
        }
        else {
            // fallback to constant intensity model, equal to average of the
            // observation
            v[0] = v[1] = v[2] = 0;
            v[3] = Atb[3] / a[15];
        }
    }
    cv::Matx<Float, 4, 4> AtA;
    cv::Vec<Float, 4> v, Atb;
    bool dirty;
};

} // namespace calibration
} // namespace orp
