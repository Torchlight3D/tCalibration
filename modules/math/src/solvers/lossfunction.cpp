#include "lossfunction.h"

#include <ceres/loss_function.h>

namespace tl {

std::unique_ptr<ceres::LossFunction> createLossFunction(LossFunctionType type,
                                                        double scale)
{
    switch (type) {
        case LossFunctionType::Trivial:
            return std::make_unique<ceres::TrivialLoss>();
        case LossFunctionType::Huber:
            return std::make_unique<ceres::HuberLoss>(scale);
        case LossFunctionType::SoftLOne:
            return std::make_unique<ceres::SoftLOneLoss>(scale);
        case LossFunctionType::Cauchy:
            return std::make_unique<ceres::CauchyLoss>(scale);
        case LossFunctionType::Arctan:
            return std::make_unique<ceres::ArctanLoss>(scale);
        case LossFunctionType::Tukey:
            return std::make_unique<ceres::TukeyLoss>(scale);
        default:
            LOG(FATAL) << "Invalid Loss Function of type index: " << (int)type;
            break;
    }

    return nullptr;
}

} // namespace tl
