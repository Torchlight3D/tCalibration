#include "findfundamental.h"

#include <limits>

#include <Eigen/Geometry>

#include <tMath/Ransac/RansacModelEstimator>
#include <tMath/Solvers/LossFunction>
#include <tMvs/Feature>
#include <tMvs/ViewPairInfo>
#include <tMvs/BA/BundleAdjustment>

#include "basics.h"
#include "findfundamentaleightpoints.h"
#include "triangulation.h"

namespace tl {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {

inline void normalizeCenteredFeature(Feature2D2D& feat, double focal1,
                                     double focal2)
{
    feat.feature1.pos /= focal1;
    feat.feature2.pos /= focal2;
}

inline void normalizeCenteredFeatures(std::vector<Feature2D2D>& corrs,
                                      double focal1, double focal2)
{
    for (auto& corr : corrs) {
        normalizeCenteredFeature(corr, focal1, focal2);
    }
}

} // namespace

// An estimator for computing the relative pose from 8 feature correspondences
// (via decomposition of the fundamental matrix).
//
// NOTE: Feature correspondences must be in pixel coordinates with the principal
// point removed i.e. principal point at (0, 0). This also assumes negligible
// skew (which is reasonable for most cameras).
class FundamentalEstimator
    : public RansacModelEstimator<Feature2D2D, UncalibratedRelativePose>
{
    using Base = RansacModelEstimator<Feature2D2D, UncalibratedRelativePose>;

public:
    FundamentalEstimator(double minFocal, double maxFoal)
        : _minFocal(minFocal), _maxFocal(maxFoal)
    {
    }

    size_t SampleSize() const override { return 8; }

    bool EstimateModel(
        const std::vector<Feature2D2D>& corrs,
        std::vector<UncalibratedRelativePose>* results) const override
    {
        if (corrs.size() < SampleSize()) {
            return false;
        }

        std::vector<Vector2d> imgPoints1, imgPoints2;
        imgPoints1.reserve(SampleSize());
        imgPoints2.reserve(SampleSize());
        for (size_t i{0}; i < SampleSize(); i++) {
            imgPoints1.push_back(corrs[i].feature1.pos);
            imgPoints2.push_back(corrs[i].feature2.pos);
        }

        UncalibratedRelativePose result;
        if (!FindFundamentalEightPoints(imgPoints1, imgPoints2,
                                        &result.fmatrix)) {
            return false;
        }

        // Only consider fmatrices that we can decompose focal lengths from.
        if (!FocalLengthsFromFundamentalMatrix(result.fmatrix.data(),
                                               &result.focalLength1,
                                               &result.focalLength2)) {
            return false;
        }

        // TODO: Should we check if the focal lengths are reasonable?
        // check focal length bounds
        if (_minFocal >= 1. && _maxFocal >= 1.) {
            if (result.focalLength1 < _minFocal ||
                result.focalLength1 > _maxFocal ||
                result.focalLength2 < _minFocal ||
                result.focalLength2 > _maxFocal) {
                return false;
            }
        }

        // Compose the essential matrix from the fundamental matrix and focal
        // lengths.
        Matrix3d ematrix;
        EssentialMatrixFromFundamentalMatrix(
            result.fmatrix.data(), result.focalLength1, result.focalLength2,
            ematrix.data());

        auto normalizedCorrs = corrs;
        normalizeCenteredFeatures(normalizedCorrs, result.focalLength1,
                                  result.focalLength2);

        GetBestPoseFromEssentialMatrix(ematrix, normalizedCorrs,
                                       &result.rotation, &result.position);
        results->emplace_back(result);
        return true;
    }

    bool RefineModel(const std::vector<Feature2D2D>& corrs, double thresh,
                     UncalibratedRelativePose* pose) const override
    {
        auto corrsNorm = corrs;
        normalizeCenteredFeatures(corrsNorm, pose->focalLength1,
                                  pose->focalLength2);

        ViewPairInfo twoViewInfo;
        {
            const AngleAxisd aa{pose->rotation};
            twoViewInfo.rotation = aa.angle() * aa.axis();
        }
        twoViewInfo.position = pose->position;

        BundleAdjustment::Options opts;
        opts.max_num_iterations = 10;
        opts.loss_function_type = LossFunctionType::Huber;
        opts.robust_loss_width = thresh * 1.5;

        const auto summary =
            BundleAdjustTwoViewsAngular(opts, corrsNorm, &twoViewInfo);

        pose->position = twoViewInfo.position;
        pose->rotation = AngleAxisd{twoViewInfo.rotation.norm(),
                                    twoViewInfo.rotation.normalized()}
                             .toRotationMatrix();

        return summary.final_cost < summary.initial_cost && summary.success;
    }

    // Here Sampson error is used
    double Error(const Feature2D2D& corr,
                 const UncalibratedRelativePose& res) const override
    {
        auto corrNorm = corr;
        normalizeCenteredFeature(corrNorm, res.focalLength1, res.focalLength2);

        if (!IsTriangulatedPointInFrontOfCameras(corrNorm, res.rotation,
                                                 res.position)) {
            return std::numeric_limits<double>::max();
        }

        return SquaredSampsonDistance(res.fmatrix, corr.feature1.pos,
                                      corr.feature2.pos);
    }

private:
    // for sanity checks
    double _minFocal, _maxFocal;
    // DISALLOW_COPY_AND_ASSIGN(FundamentalEstimator);
};

bool FindFundamental(const std::vector<Feature2D2D>& matches,
                     double minFocalLength, double maxFocalLength,
                     RansacType ransacType, const SacParameters& ransacParams,
                     UncalibratedRelativePose* result,
                     SacSummary* ransacSummary)
{
    const FundamentalEstimator estimator{minFocalLength, maxFocalLength};
    auto ransac = createRansac(ransacType, ransacParams, estimator);

    return ransac->Estimate(matches, result, ransacSummary);
}

} // namespace tl
