#pragma once

#include <memory>

namespace ceres {
class LossFunction;
} // namespace ceres

namespace tl {

// For bundle adjustment we can use a robust cost function to maintain
// robustness to outliers. In particular, this function can help when feature
// tracks have outliers so that bundle adjustment may still optimize 3D points
// and camera poses properly without being catastrophically affected by
// outliers.
enum class LossFunctionType
{
    Trivial = 0,
    Huber,
    SoftLOne,
    Cauchy,
    Arctan,
    Tukey,
};

std::unique_ptr<ceres::LossFunction> createLossFunction(LossFunctionType type,
                                                        double scale);

} // namespace tl
