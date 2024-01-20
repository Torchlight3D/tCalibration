#include "EquidistantDistortion.h"
#include "FovDistortion.h"
#include "RadialTangentialDistortion.h"

namespace tl {

EquidistantDistortion::EquidistantDistortion() {}

EquidistantDistortion::EquidistantDistortion(double k1, double k2, double k3,
                                             double k4)
    : _k1(k1), _k2(k2), _k3(k3), _k4(k4)
{
}

EquidistantDistortion::~EquidistantDistortion() {}

void EquidistantDistortion::setParameters(const Eigen::MatrixXd& params)
{
    // TODO: check size
    _k1 = params(0, 0);
    _k2 = params(1, 0);
    _k3 = params(2, 0);
    _k4 = params(3, 0);
}

void EquidistantDistortion::parameters(Eigen::MatrixXd& params) const
{
    params.resize(IntrinsicsDimension, 1);
    params(0, 0) = _k1;
    params(1, 0) = _k2;
    params(2, 0) = _k3;
    params(3, 0) = _k4;
}

Eigen::Vector2i EquidistantDistortion::parameterSize() const
{
    return {IntrinsicsDimension, 1};
}

void EquidistantDistortion::clear()
{
    _k1 = 0.0;
    _k2 = 0.0;
    _k3 = 0.0;
    _k4 = 0.0;
}

void EquidistantDistortion::update(const double* v)
{
    // TODO: check size
    _k1 += v[0];
    _k2 += v[1];
    _k3 += v[2];
    _k4 += v[3];
}

int EquidistantDistortion::minimalDimensions() const
{
    return IntrinsicsDimension;
}

FovDistortion::FovDistortion() {}

FovDistortion::FovDistortion(double w) : _w(w) {}

FovDistortion::~FovDistortion() {}

void FovDistortion::setParameters(const Eigen::MatrixXd& params)
{
    // check size
    _w = params(0, 0);
}

void FovDistortion::parameters(Eigen::MatrixXd& params) const
{
    params.resize(IntrinsicsDimension, 1);
    params(0, 0) = _w;
}

Eigen::Vector2i FovDistortion::parameterSize() const
{
    return {IntrinsicsDimension, 1};
}

void FovDistortion::clear() { _w = 1.0; }

void FovDistortion::update(const double* v)
{
    // check size
    _w += v[0];
}

int FovDistortion::minimalDimensions() const { return IntrinsicsDimension; }

RadialTangentialDistortion::RadialTangentialDistortion() {}

RadialTangentialDistortion::RadialTangentialDistortion(double k1, double k2,
                                                       double p1, double p2)
    : _k1(k1), _k2(k2), _p1(p1), _p2(p2)
{
}

RadialTangentialDistortion::~RadialTangentialDistortion() {}

void RadialTangentialDistortion::setParameters(const Eigen::MatrixXd& params)
{
    // check size
    _k1 = params(0, 0);
    _k2 = params(1, 0);
    _p1 = params(2, 0);
    _p2 = params(3, 0);
}

void RadialTangentialDistortion::parameters(Eigen::MatrixXd& params) const
{
    params.resize(IntrinsicsDimension, 1);
    params(0, 0) = _k1;
    params(1, 0) = _k2;
    params(2, 0) = _p1;
    params(3, 0) = _p2;
}

Eigen::Vector2i RadialTangentialDistortion::parameterSize() const
{
    return {IntrinsicsDimension, 1};
}

void RadialTangentialDistortion::clear()
{
    _k1 = 0.0;
    _k2 = 0.0;
    _p1 = 0.0;
    _p2 = 0.0;
}

void RadialTangentialDistortion::update(const double* v)
{
    _k1 += v[0];
    _k2 += v[1];
    _p1 += v[2];
    _p2 += v[3];
}

int RadialTangentialDistortion::minimalDimensions() const
{
    return IntrinsicsDimension;
}

} // namespace tl
