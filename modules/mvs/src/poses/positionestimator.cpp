#include "positionestimator.h"

#include <glog/logging.h>

#include "linearpositionestimator.h"
#include "ligtpositionestimator.h"
#include "ludpositionestimator.h"
#include "nonlinearpositionestimator.h"

namespace tl {

bool EstimateGlobalPosition(
    GlobalPositionEstimatorType type,
    const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientation,
    std::unordered_map<ViewId, Eigen::Vector3d>* positions)
{
    // ...
    return false;
}

} // namespace tl
