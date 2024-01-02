#include "RollingShutter.h"

namespace thoht {

RollingShutter::RollingShutter() : _lineDelay(0.0) {}

RollingShutter::RollingShutter(double lineDelay) : _lineDelay(lineDelay) {}

RollingShutter::~RollingShutter() {}

void RollingShutter::update(const double* v) { _lineDelay += v[0]; }

int RollingShutter::minimalDimensions() const { return 1; }

void RollingShutter::setParameters(const Eigen::MatrixXd& params)
{
    // TODO: check size
    _lineDelay = params(0, 0);
}

void RollingShutter::getParameters(Eigen::MatrixXd& params) const
{
    params.resize(1, 1);
    params(0, 0) = _lineDelay;
}

Eigen::Vector2i RollingShutter::parameterSize() const { return {1, 1}; }

} // namespace thoht
