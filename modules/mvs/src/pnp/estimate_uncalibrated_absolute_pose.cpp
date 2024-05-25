#include "estimate_uncalibrated_absolute_pose.h"

#include <ceres/rotation.h>

#include <tCamera/CameraIntrinsics>
#include <tCamera/CameraMatrixUtils>
#include <tMath/Eigen/Types>
#include <tMath/RANSAC/RansacCreator>
#include <tMath/RANSAC/RansacModelEstimator>
#include <tMvs/Feature>
#include <tMvs/PnP/P4PFocalLength>

namespace tl {

using Eigen::Vector2d;

namespace {

} // namespace

/// ======= UncalibratedAbsolutePoseEstimator starts from here
// An estimator for computing the uncalibrated absolute pose from 4 feature
// correspondences. The feature correspondences should be normalized such that
// the principal point is at (0, 0).
class UncalibratedAbsolutePoseEstimator
    : public Estimator<Feature2D3D, Matrix34d>
{
    using Base = Estimator<Feature2D3D, Matrix34d>;

public:
    using Base::Base;

    // 3 correspondences are needed to determine the absolute pose.
    int SampleSize() const override { return 4; }

    // Estimates candidate absolute poses from correspondences.
    bool EstimateModel(const std::vector<Feature2D3D>& corres,
                       std::vector<Matrix34d>* poses) const override
    {
        const Vector2dList point2s{corres[0].feature, corres[1].feature,
                                   corres[2].feature, corres[3].feature};
        const Vector3dList point3s{corres[0].world_point, corres[1].world_point,
                                   corres[2].world_point,
                                   corres[3].world_point};

        const int solutions =
            FourPointPoseAndFocalLength(point2s, point3s, *poses);
        return solutions > 0;
    }

    // The error for a correspondences given an absolute pose. This is the
    // squared reprojection error.
    double Error(const Feature2D3D& corr, const Matrix34d& pose) const override
    {
        // T = [ R, t ]
        // [x, 1]' = T * [X, 1]'
        const Vector2d reprojected =
            (pose * corr.world_point.homogeneous()).eval().hnormalized();
        return (reprojected - corr.feature).squaredNorm();
    }
};

/// ======= EstimateUncalibratedAbsolutePose starts from here
bool EstimateUncalibratedAbsolutePose(
    const SacParameters& ransacParams, RansacType ransacType,
    const std::vector<Feature2D3D>& normalized_correspondences,
    UncalibratedAbsolutePose* pose, SacSummary* sacSummary)
{
    UncalibratedAbsolutePoseEstimator estimator;
    auto ransac = createRansac(ransacType, ransacParams, estimator);

    // Estimate the absolute pose.
    Matrix34d K;
    const bool success =
        ransac->Estimate(normalized_correspondences, &K, sacSummary);

    // Recover the focal length and pose.
    Eigen::Matrix3d P;
    Eigen::Vector3d rvec;
    decomposeProjectionMatrix(K, P, rvec, pose->position);

    ceres::AngleAxisToRotationMatrix(
        rvec.data(), ceres::ColumnMajorAdapter3x3(pose->rotation.data()));

    pose->focal_length = P(0, 0) / P(2, 2);

    return success;
}

} // namespace tl
