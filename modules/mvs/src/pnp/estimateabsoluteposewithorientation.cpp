#include "estimateabsoluteposewithorientation.h"

#include <ceres/rotation.h>
#include <Eigen/Geometry>

#include <tMath/Ransac/RansacModelEstimator>
#include <tMath/Ransac/RansacCreator>
#include <tMath/Eigen/Rotation>
#include <tMvs/Feature>

namespace tl {

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {

// Rotate the correspondences to be in the world space instead of camera
// coordinates.
void RotateCorrespondences(
    const std::vector<Feature2D3D>& normalized_correspondences,
    const Eigen::Vector3d& camera_orientation,
    std::vector<Feature2D3D>* rotated_correspondences)
{
    Matrix3d camera_to_world_rotation;
    ceres::AngleAxisToRotationMatrix(
        camera_orientation.data(),
        ceres::ColumnMajorAdapter3x3(camera_to_world_rotation.data()));
    camera_to_world_rotation.transposeInPlace();

    rotated_correspondences->resize(normalized_correspondences.size());
    for (int i = 0; i < normalized_correspondences.size(); i++) {
        const Vector3d feature_ray =
            camera_to_world_rotation *
            normalized_correspondences[i].feature.homogeneous();
        (*rotated_correspondences)[i].feature = feature_ray.hnormalized();
        (*rotated_correspondences)[i].world_point =
            normalized_correspondences[i].world_point;
    }
}

// An estimator for computing the absolute pose from 2 feature
// correspondences. The feature correspondences should be normalized by the
// focal length with the principal point at (0, 0) and rotated into the same
// coordinate system as the points.
class AbsolutePoseWithKnownOrientationEstimator
    : public RansacModelEstimator<Feature2D3D, Eigen::Vector3d>
{
    using Base = RansacModelEstimator<Feature2D3D, Eigen::Vector3d>;

public:
    using Base::Base;

    // 2 correspondences are needed to determine the absolute position.
    size_t SampleSize() const override { return 2; }

    bool EstimateModel(const std::vector<Feature2D3D>& corrs,
                       std::vector<Eigen::Vector3d>* positions) const override
    {
        if (corrs.size() < SampleSize()) {
            return false;
        }

        Vector3d position;
        if (!PositionFromTwoRays(corrs[0].feature, corrs[0].world_point,
                                 corrs[1].feature, corrs[1].world_point,
                                 &position)) {
            return false;
        }

        positions->push_back(position);
        return true;
    }

    bool RefineModel(const std::vector<Feature2D3D>& corrs, double threshold,
                     Eigen::Vector3d* position) const override
    {
        return true;
    }

    // The error for a correspondences given an absolute position. This is the
    // squared reprojection error.
    double Error(const Feature2D3D& corr,
                 const Eigen::Vector3d& position) const override
    {
        // The reprojected point is computed as X - c where c is the position,
        // and X is the 3D point.
        const Vector2d reprojected_feature =
            (corr.world_point - position).hnormalized();
        return (reprojected_feature - corr.feature).squaredNorm();
    }

private:
    // DISALLOW_COPY_AND_ASSIGN(AbsolutePoseWithKnownOrientationEstimator);
};

} // namespace

// Estimates the absolute pose with known orientation. It is assumed that the 2D
// features in the 2D-3D correspondences are normalized by the camera
// intrinsics. Returns true if the position could be successfully estimated and
// false otherwise. The quality of the result depends on the quality of the
// input data.
bool EstimateAbsolutePoseWithKnownOrientation(
    const SacParameters& ransac_params, RansacType ransac_type,
    const Eigen::Vector3d& camera_orientation,
    const std::vector<Feature2D3D>& normalized_correspondences,
    Eigen::Vector3d* camera_position, SacSummary* ransac_summary)
{
    std::vector<Feature2D3D> rotated_correspondences;
    RotateCorrespondences(normalized_correspondences, camera_orientation,
                          &rotated_correspondences);

    AbsolutePoseWithKnownOrientationEstimator estimator;
    auto ransac = createRansac(ransac_type, ransac_params, estimator);

    return ransac->Estimate(rotated_correspondences, camera_position,
                            ransac_summary);
}

namespace EstimateAbsolutePose {

bool WithKnownOrientation(const std::vector<Feature2D3D>& correspondences,
                          const Eigen::Vector3d& orientation,
                          RansacType ransacType,
                          const SacParameters& ransacParams,
                          Eigen::Vector3d* position, SacSummary* ransacSummary)
{
    std::vector<Feature2D3D> rotated_correspondences;
    RotateCorrespondences(correspondences, orientation,
                          &rotated_correspondences);

    AbsolutePoseWithKnownOrientationEstimator estimator;
    auto ransac = createRansac(ransacType, ransacParams, estimator);

    return ransac->Estimate(rotated_correspondences, position, ransacSummary);
}

} // namespace EstimateAbsolutePose

} // namespace tl
