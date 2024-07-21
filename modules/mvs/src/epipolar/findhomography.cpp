#include "findhomography.h"

#include <Eigen/Geometry>

#include <tMath/Ransac/RansacModelEstimator>
#include <tMvs/Feature>
#include <tMvs/BA/BundleAdjustment>

#include "findhomographyfourpoints.h"

namespace tl {

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {

// An estimator for computing a homography from 4 feature correspondences. The
// feature correspondences should be normalized by the focal length with the
// principal point at (0, 0).
class HomographyEstimator
    : public RansacModelEstimator<Feature2D2D, Eigen::Matrix3d>
{
    using Base = RansacModelEstimator<Feature2D2D, Eigen::Matrix3d>;

public:
    using Base::Base;

    size_t SampleSize() const override { return 4; }

    bool EstimateModel(
        const std::vector<Feature2D2D>& corrs,
        std::vector<Eigen::Matrix3d>* homographies) const override
    {
        if (corrs.size() < SampleSize()) {
            return false;
        }

        std::vector<Vector2d> imgPoints1(SampleSize());
        std::vector<Vector2d> imgPoints2(SampleSize());
        for (size_t i{0}; i < SampleSize(); i++) {
            imgPoints1[i] = corrs[i].feature1.pos;
            imgPoints2[i] = corrs[i].feature2.pos;
        }

        Matrix3d homography;
        if (!FourPointHomography(imgPoints1, imgPoints2, &homography)) {
            return false;
        }

        homographies->emplace_back(homography);
        return true;
    }

    bool RefineModel(const std::vector<Feature2D2D>& correspondences,
                     double thresh, Eigen::Matrix3d* homography) const override
    {
        BundleAdjustment::Options opts;
        opts.max_num_iterations = 15;
        opts.loss_function_type = LossFunctionType::Truncated;
        opts.verbose = false;
        opts.robust_loss_width = thresh;

        const auto summary =
            OptimizeHomography(opts, correspondences, homography);

        return summary.final_cost < summary.initial_cost;
    }

    // The error for a correspondences given a model. This is the asymmetric
    // distance that measures reprojection error in one image.
    double Error(const Feature2D2D& correspondence,
                 const Eigen::Matrix3d& homography) const override
    {
        const Vector3d reprojected_point =
            homography * correspondence.feature1.pos.homogeneous();
        return (correspondence.feature2.pos - reprojected_point.hnormalized())
            .squaredNorm();
    }

private:
    // DISALLOW_COPY_AND_ASSIGN(HomographyEstimator);
};

} // namespace

bool EstimateHomography(const SacParameters& ransacParams,
                        RansacType ransacType,
                        const std::vector<Feature2D2D>& corrs,
                        Eigen::Matrix3d* homography, SacSummary* ransacSummary)
{
    HomographyEstimator estimator;
    auto ransac = createRansac(ransacType, ransacParams, estimator);

    return ransac->Estimate(corrs, homography, ransacSummary);
}

bool FindHomography(const std::vector<Feature2D2D>& corrs,
                    RansacType ransacType, const SacParameters& ransacParams,
                    Eigen::Matrix3d* homography, SacSummary* ransacSummary)
{
    HomographyEstimator estimator;
    auto ransac = createRansac(ransacType, ransacParams, estimator);

    return ransac->Estimate(corrs, homography, ransacSummary);
}

} // namespace tl
