#include "rotationestimator.h"

#include "hybridrotationestimator.h"
#include "lagrangedualrotationestimator.h"
#include "linearrotationestimator.h"
#include "nonlinearrotationestimator.h"
#include "robustrotationestimator.h"

namespace tl {

bool EstimateGlobalRotations(
    GlobalRotationEstimatorType type,
    const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
    std::unordered_map<ViewId, Eigen::Vector3d>* rotations)
{
    std::unique_ptr<RotationEstimator> estimator;
    switch (type) {
        case GlobalRotationEstimatorType::Robust: {
            RobustRotationEstimator::Options opts;
            estimator.reset(new RobustRotationEstimator(opts));
            break;
        }
        case GlobalRotationEstimatorType::Nonlinear: {
            estimator.reset(new NonlinearRotationEstimator());
            break;
        }
        case GlobalRotationEstimatorType::Linear: {
            estimator.reset(new LinearRotationEstimator());
            break;
        }
        case GlobalRotationEstimatorType::LagrangeDuality: {
            estimator.reset(new LagrangeDualRotationEstimator());
            break;
        }
        case GlobalRotationEstimatorType::Hybrid: {
            estimator.reset(new HybridRotationEstimator());
            break;
        }
        default: {
            LOG(FATAL) << "Unsupported global rotations estimation algorithm.";
        }
    }

    return estimator->EstimateRotations(viewPairs, rotations);
}

} // namespace tl
