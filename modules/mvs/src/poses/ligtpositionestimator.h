#pragma once

#include <Eigen/SparseCore>

#include "positionestimator.h"

namespace tl {

class Scene;
class View;
struct ViewPairInfo;

// Brief:
// Estimates the camera position of views given global orientations and
// normalized image correspondeces of the same scene point across image
// triplets. The constraints formed by each triplet are used to create a sparse
// linear system to solve for the positions.
//
// Ref:
// "A Pose-only Solution to Visual Reconstruction and Navigation" by Cai Qi,
// Zhang Lilian, Wu Yuanxin, Yu Wenxian and Hu Dewen (PAMI2023)
class LiGTPositionEstimator final : public PositionEstimator
{
public:
    struct Options
    {
        int num_threads = 1;

        // Maximum number of inverse power iterations to perform while
        // extracting the eigenvector corresponding to the smallest eigenvalue.
        int max_power_iterations = 1000;

        // The threshold at which to the iterative eigensolver method is
        // considered to be converged.
        double eigensolver_threshold = 1e-8;

        // threshold for using sparse solver instead of SVD
        int max_num_views_svd = 500;
    };

    LiGTPositionEstimator(const Options& opts, const Scene& scene);
    ~LiGTPositionEstimator();

    bool EstimatePositions(
        const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
        const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
        std::unordered_map<ViewId, Eigen::Vector3d>* positions) override;

private:
    void FindTripletsForTracks();

    // Eq.29. Get best base views for point
    std::pair<ViewId, ViewId> GetBestBaseViews(TrackId id) const;

    using BCD = std::tuple<Eigen::Matrix3d, Eigen::Matrix3d, Eigen::Matrix3d>;
    BCD calcBCDForTrack(ViewId viewId1, ViewId viewId2, ViewId viewId3,
                        TrackId trackId) const;

    // Sets up the linear system with the constraints that each triplet adds.
    void CreateLinearSystem(Eigen::SparseMatrix<double>* constraint_matrix);

    // Positions are estimated from an eigenvector that is unit-norm with an
    // ambiguous sign. To ensure that the sign of the camera positions is
    // correct, we measure the relative translations from estimated camera
    // positions and compare that to the relative positions. If the sign is
    // incorrect, we flip the sign of all camera positions.
    void FlipSignOfPositionsIfNecessary(
        std::unordered_map<ViewId, Eigen::Vector3d>* positions);

private:
    const Options _opts;
    const Scene& _scene;
    const std::unordered_map<ViewIdPair, ViewPairInfo>* _viewPairs;
    std::unordered_map<ViewId, Eigen::Matrix3d> _orientations;

    // Eq.29. save a view triplet for each track
    std::unordered_map<TrackId, std::vector<ViewIdTriplet>> _trackIdToTriplets;
    std::unordered_map<TrackId, std::vector<BCD>> _BCDs;
    std::unordered_map<ViewId, int> _viewIdToNumTriplets;
    std::unordered_map<ViewId, int> linear_system_index_;

    // DISALLOW_COPY_AND_ASSIGN(LiGTPositionEstimator);
};

} // namespace tl
