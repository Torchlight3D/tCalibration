#include "distribution.h"

#include <glog/logging.h>

#include <tCore/Math>

namespace tl {

using std::numbers::sqrt2;

NormalDistribution::NormalDistribution(double mean, double sigma) : mean_(mean)
{
    CHECK_GT(sigma, 0.)
        << "Sigma must be greater than zero in a normal distribution";
    alpha_ = 1. / (sigma * sqrt2 * std::sqrt(pi));
    beta_ = -1. / (2. * sigma * sigma);
}

double NormalDistribution::Eval(double x) const
{
    const double normalized_x = x - mean_;
    return alpha_ * exp(beta_ * normalized_x * normalized_x);
}

UniformDistribution::UniformDistribution(double left, double right)
    : left_(left), right_(right)
{
    CHECK_LT(left, right) << "Left bound must be less than the right bound for "
                          << "uniform distributions.";
    CHECK_NE(right, left) << "Left bound is equal to right bound! Uniform "
                          << "distribution must have a nonzero range.";
    inverse_span_ = (left == right) ? 1. : 1. / (right - left);
}

double UniformDistribution::Eval(double x) const
{
    return (left_ <= x && x <= right_) ? inverse_span_ : 0.;
}

} // namespace tl
