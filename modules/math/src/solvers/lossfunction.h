#pragma once

#include <memory>

#include <tMath/Types>

namespace ceres {
class LossFunction;
} // namespace ceres

namespace tl {

std::unique_ptr<ceres::LossFunction> createLossFunction(LossFunctionType type,
                                                        double scale);

} // namespace tl
