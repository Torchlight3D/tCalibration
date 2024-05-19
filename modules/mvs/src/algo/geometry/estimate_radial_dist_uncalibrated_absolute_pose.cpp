﻿#include "estimate_radial_dist_uncalibrated_absolute_pose.h"

#include <Eigen/Dense>

#include <tMath/RANSAC/RansacCreator>
#include <tMath/RANSAC/RansacModelEstimator>
#include <tMvs/P4PFocalLengthDistortion>
#include <tMvs/FeatureCorrespondence>

namespace tl {

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {

void DistortPoint(const Vector2d& point2d, double distortion,
                  Vector2d* distorted_point)
{
    const double rr = point2d.squaredNorm();

    const double denom = 2.0 * distortion * rr;
    const double inner_sqrt = 1.0 - 4.0 * distortion * rr;

    // If the denominator is nearly zero then we can evaluate the distorted
    // coordinates as k or r_u^2 goes to zero. Both evaluate to the identity.
    if (std::abs(denom) < 1e-15 || inner_sqrt < 0.0) {
        (*distorted_point)[0] = point2d[0];
        (*distorted_point)[1] = point2d[1];
    }
    else {
        const double scale = (1.0 - std::sqrt(inner_sqrt)) / denom;
        (*distorted_point)[0] = point2d[0] * scale;
        (*distorted_point)[1] = point2d[1] * scale;
    }
}

} // namespace

// An estimator for computing the uncalibrated absolute pose from 4 feature
// correspondences. The feature correspondences should be normalized such that
// the principal point is at (0, 0).
class RadialDistUncalibratedAbsolutePoseEstimator
    : public Estimator<FeatureCorrespondence2D3D,
                       RadialDistUncalibratedAbsolutePose>
{
public:
    using Estimator<FeatureCorrespondence2D3D,
                    RadialDistUncalibratedAbsolutePose>::Estimator;

    // 4 correspondences are needed to determine the absolute pose.
    double SampleSize() const override { return 4; }

    // Estimates candidate absolute poses from correspondences.
    bool EstimateModel(
        const std::vector<FeatureCorrespondence2D3D>& corres,
        std::vector<RadialDistUncalibratedAbsolutePose>* poses) const override
    {
        const Vector2dList point2s{corres[0].feature, corres[1].feature,
                                   corres[2].feature, corres[3].feature};
        const Vector3dList point3s{corres[0].world_point, corres[1].world_point,
                                   corres[2].world_point,
                                   corres[3].world_point};

        Matrix3dList rotations;
        Vector3dList translations;
        std::vector<double> radialDistortions;
        std::vector<double> focalLenghths;
        if (!FourPointsPoseFocalLengthRadialDistortion(
                point2s, point3s,
                {opts_.min_focal_length, opts_.max_focal_length,
                 opts_.min_radial_distortion, opts_.max_radial_distortion},
                &rotations, &translations, &radialDistortions,
                &focalLenghths)) {
            return false;
        }

        poses->resize(rotations.size());
        for (size_t i{0}; i < rotations.size(); ++i) {
            (*poses)[i].radial_distortion = radialDistortions[i];
            (*poses)[i].focal_length = focalLenghths[i];
            (*poses)[i].rotation = rotations[i];
            (*poses)[i].translation = translations[i];
        }

        return !rotations.empty();
    }

    // The error for a correspondences given an absolute pose. This is the
    // squared reprojection error.
    double Error(const FeatureCorrespondence2D3D& corr,
                 const RadialDistUncalibratedAbsolutePose& pose) const override
    {
        if (pose.translation[2] < 0.) {
            return 1e10;
        }

        // undistort the feature with the estimated radial distortion parameter
        // project der world point with the given focal length and
        // compare it to the undistorted image point
        const Matrix3d K =
            Vector3d(pose.focal_length, pose.focal_length, 1.).asDiagonal();
        const Vector3d pt =
            (pose.rotation * corr.world_point + pose.translation);
        const Vector2d pt_u = (K * pt).hnormalized();
        Vector2d pt_d;
        DistortPoint(pt_u, pose.radial_distortion, &pt_d);

        return (pt_d - corr.feature).squaredNorm();
    }

    void SetMetadata(const RadialDistUncalibratedAbsolutePoseMetaData& options)
    {
        opts_ = options;
    }

private:
    RadialDistUncalibratedAbsolutePoseMetaData opts_;
};

bool EstimateRadialDistUncalibratedAbsolutePose(
    const SacParameters& sacParams, RansacType sacType,
    const std::vector<FeatureCorrespondence2D3D>& corres,
    const RadialDistUncalibratedAbsolutePoseMetaData& options,
    RadialDistUncalibratedAbsolutePose* pose, SacSummary* sacSummary)
{
    RadialDistUncalibratedAbsolutePoseEstimator estimator;
    estimator.SetMetadata(options);

    auto ransac = createRansac(sacType, sacParams, estimator);
    const bool success = ransac->Estimate(corres, pose, sacSummary);

    return success;
}

} // namespace tl
