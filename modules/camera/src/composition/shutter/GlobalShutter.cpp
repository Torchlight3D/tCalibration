#include "GlobalShutter.h"

namespace tl {

GlobalShutter::GlobalShutter() {}

GlobalShutter::~GlobalShutter() {}

void GlobalShutter::update(const double *) {}

int GlobalShutter::minimalDimensions() const { return 0; }

void GlobalShutter::getParameters(Eigen::MatrixXd &params) const
{
    params.resize(0, 0);
}

void GlobalShutter::setParameters(const Eigen::MatrixXd & /* params */) {}

Eigen::Vector2i GlobalShutter::parameterSize() const { return {0, 0}; }

} // namespace tl
