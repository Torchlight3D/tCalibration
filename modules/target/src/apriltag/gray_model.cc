#include "gray_model.h"

#include <iostream>

#include <Eigen/Dense>
#include <Eigen/LU>
#include <glog/logging.h>

namespace apriltags {

GrayModel::GrayModel() : _A(), _v(), _b(), _nobs(0), _dirty(false)
{
    _A.setZero();
    _v.setZero();
    _b.setZero();
}

void GrayModel::addObservation(float x, float y, float gray)
{
    const float xy = x * y;

    // update only upper-right elements. A'A is symmetric,
    // we'll fill the other elements in later.
    _A(0, 0) += x * x;
    _A(0, 1) += x * y;
    _A(0, 2) += x * xy;
    _A(0, 3) += x;
    _A(1, 1) += y * y;
    _A(1, 2) += y * xy;
    _A(1, 3) += y;
    _A(2, 2) += xy * xy;
    _A(2, 3) += xy;
    _A(3, 3) += 1;

    _b[0] += x * gray;
    _b[1] += y * gray;
    _b[2] += xy * gray;
    _b[3] += gray;

    _nobs++;
    _dirty = true;
}

float GrayModel::interpolate(float x, float y)
{
    if (_dirty) {
        compute();
    }

    return _v[0] * x + _v[1] * y + _v[2] * x * y + _v[3];
}

void GrayModel::compute()
{
    // we really only need 4 linearly independent observations to fit our
    // answer, but we'll be very sensitive to noise if we don't have an
    // over-determined system. Thus, require at least 6 observations (or we'll
    // use a constant model below).

    _dirty = false;
    if (_nobs >= 6) {
        // make symmetric
        for (int i = 0; i < 4; i++) {
            for (int j = i + 1; j < 4; j++) {
                _A(j, i) = _A(i, j);
            }
        }

        Eigen::Matrix4d Ainv;
        double determinant;
        bool invertible;
        _A.computeInverseAndDetWithCheck(Ainv, determinant, invertible);
        if (invertible) {
            _v = Ainv * _b;
            return;
        }

        LOG(ERROR) << "AprilTags::GrayModel::compute() has underflow in matrix "
                      "inverse";
    }

    // If we get here, either nobs < 6 or the matrix inverse generated
    // an underflow, so use a constant model.
    _v.setZero(); // need the cast to avoid operator= ambiguity wrt. const-ness
    _v[3] = _b[3] / _nobs;
}

} // namespace apriltags
