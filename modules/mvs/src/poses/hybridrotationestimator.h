#pragma once

#include <memory>

#include "lagrangedualrotationestimator.h"
#include "irlsposerefine.h"
#include "rotationestimator.h"

namespace tl {

class HybridRotationEstimator : public RotationEstimator
{
public:
    struct Options
    {
        math::SDPSolver::Options sdp_solver_options;
        IRLSRotationLocalRefiner::Options irls_options;

        Options() {}
    };

    explicit HybridRotationEstimator(const Options& options = {});

    bool EstimateRotations(
        const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
        std::unordered_map<ViewId, Eigen::Vector3d>* rotations) override;

private:
    void GlobalRotationsToTangentSpace(
        const std::unordered_map<ViewId, Eigen::Vector3d>& global_rotations,
        Eigen::VectorXd* tangent_space_step);

private:
    Options options_;

    // number of images/frames
    size_t images_num_;

    int dim_;

    // this hash table is used for non-continuous index, such as
    // unordered internet datasets that composed of many unconnected components
    std::unordered_map<ViewId, int> view_id_to_index_;

    std::unique_ptr<LagrangeDualRotationEstimator> ld_rotation_estimator_;
    std::unique_ptr<IRLSRotationLocalRefiner> irls_rotation_refiner_;
};

} // namespace tl
