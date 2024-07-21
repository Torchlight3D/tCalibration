#include "lossfunction.h"

namespace tl {

TruncatedLoss::TruncatedLoss(double a) : squared_error_(a * a) {}

void TruncatedLoss::Evaluate(double s, double rho[3]) const
{
    rho[0] = std::min(s, squared_error_);
    rho[1] = s < squared_error_ ? 1. : 0.;
    rho[2] = 0.;
}

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
