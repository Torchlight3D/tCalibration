#include "findessential.h"

#include <limits>
#include <memory>

#include <ceres/types.h>
#include <Eigen/Geometry>

#include <tMath/Ransac/RansacModelEstimator>
#include <tMvs/Feature>
#include <tMvs/ViewPairInfo>
#include <tMvs/BA/BundleAdjustment>

#include "basics.h"
#include "findessentialfivepoints.h"
#include "triangulation.h"

namespace tl {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {

// An estimator for computing the relative pose from 5 feature
// correspondences. The feature correspondences should be normalized
// by the focal length with the principal point at (0, 0).
class RelativePoseEstimator
    : public RansacModelEstimator<Feature2D2D, RelativePose>
{
    using Base = RansacModelEstimator<Feature2D2D, RelativePose>;

public:
    using Base::Base;

    size_t SampleSize() const override { return 5; }

    // Estimates candidate relative poses from corrs.
    bool EstimateModel(const std::vector<Feature2D2D>& corrs,
                       std::vector<RelativePose>* poses) const override
    {
        if (corrs.size() < SampleSize()) {
            return false;
        }

        const auto pointCount = corrs.size();

        std::vector<Vector2d> imaPoints1, imgPoints2;
        imaPoints1.reserve(pointCount);
        imgPoints2.reserve(pointCount);
        for (const auto& corr : corrs) {
            imaPoints1.push_back(corr.feature1.pos);
            imgPoints2.push_back(corr.feature2.pos);
        }

        std::vector<Matrix3d> ematrices;
        if (!FivePointRelativePose(imaPoints1, imgPoints2, &ematrices)) {
            return false;
        }

        poses->reserve(ematrices.size() * 4);
        for (const auto& ematrix : ematrices) {
            RelativePose pose;
            pose.essential_matrix = ematrix;

            // The best relative pose decomposition should have at least 4
            // triangulated points in front of the camera. This is because one
            // point may be at infinity.
            if (const auto pointCountInFrontOfCamera =
                    GetBestPoseFromEssentialMatrix(
                        ematrix, corrs, &pose.rotation, &pose.position);
                pointCountInFrontOfCamera >= 4) {
                poses->push_back(pose);
            }
        }

        return poses->size() > 0;
    }

    bool RefineModel(const std::vector<Feature2D2D>& corrs, double thresh,
                     RelativePose* pose) const override
    {
        ViewPairInfo twoViewInfo;
        const AngleAxisd aa_init{pose->rotation};
        twoViewInfo.rotation = aa_init.angle() * aa_init.axis();
        twoViewInfo.position = pose->position;

        BundleAdjustment::Options opts;
        opts.max_num_iterations = 15;
        opts.linear_solver_type = ceres::CGNR;
        opts.preconditioner_type = ceres::JACOBI;
        opts.loss_function_type = LossFunctionType::Truncated;
        opts.verbose = false;
        opts.robust_loss_width = thresh;

        const auto summary =
            BundleAdjustTwoViewsAngular(opts, corrs, &twoViewInfo);

        pose->position = twoViewInfo.position;
        AngleAxisd aa_opt;
        aa_opt.angle() = twoViewInfo.rotation.norm();
        aa_opt.axis() = twoViewInfo.rotation / aa_opt.angle();
        pose->rotation = aa_opt.toRotationMatrix();
        return summary.final_cost < summary.initial_cost;
    }

    // The error for a correspondences given a model. This is the squared
    // sampson error.
    double Error(const Feature2D2D& corr,
                 const RelativePose& pose) const override
    {
        if (IsTriangulatedPointInFrontOfCameras(corr, pose.rotation,
                                                pose.position)) {
            return SquaredSampsonDistance(pose.essential_matrix,
                                          corr.feature1.pos, corr.feature2.pos);
        }

        return std::numeric_limits<double>::max();
    }

private:
    // DISALLOW_COPY_AND_ASSIGN(RelativePoseEstimator);
};

} // namespace

bool EstimateRelativePose(const SacParameters& ransacParams,
                          RansacType ransacType,
                          const std::vector<Feature2D2D>& corrs,
                          RelativePose* pose, SacSummary* ransacSummary)
{
    RelativePoseEstimator estimator;
    auto ransac = createRansac(ransacType, ransacParams, estimator);

    return ransac->Estimate(corrs, pose, ransacSummary);
}

bool FindEssential(const std::vector<Feature2D2D>& corrs, RansacType ransacType,
                   const SacParameters& ransacParams, RelativePose* pose,
                   SacSummary* ransacSummary)
{
    RelativePoseEstimator estimator;
    auto ransac = createRansac(ransacType, ransacParams, estimator);

    return ransac->Estimate(corrs, pose, ransacSummary);
}

} // namespace tl
