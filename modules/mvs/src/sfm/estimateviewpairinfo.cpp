#include "estimateviewpairinfo.h"

#include <Eigen/Geometry>
#include <glog/logging.h>

#include <tCamera/Camera>
#include <tMvs/Epipolar/Basics>
#include <tMvs/Epipolar/FindEssential>
#include <tMvs/Epipolar/FindFundamental>

#include "../desc/feature.h"
#include "../desc/viewpairinfo.h"
#include "visibilitypyramid.h"

namespace tl {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {

// Normalizes the points in image feature matches with camera intrinsics by
// removing the projective effect.
std::vector<Feature2D2D> normalizeMatches(
    const CameraMetaData& meta1, const CameraMetaData& meta2,
    const std::vector<Feature2D2D>& matches)
{
    Camera camera1, camera2;
    camera1.setFromMetaData(meta1);
    camera2.setFromMetaData(meta2);

    // Even if no focal length prior is given, the Camera::setFromMetaData()
    // will still set the focal length to a reasonable guess. However, for
    // cameras with no focal length prior we DONT want the following feature
    // normalization to divide the focal length. We set the focal lengths to 1.0
    // so that both of known and unknown focal length follow the same procedure.
    if (!meta1.focalLength.has_value() || !meta2.focalLength.has_value()) {
        camera1.setFocalLength(1.);
        camera2.setFocalLength(1.);
    }

    std::vector<Feature2D2D> normMatches;
    for (const auto& match : matches) {
        Feature2D2D normMatch;
        normMatch.feature1 =
            Feature{camera1.pixelToNormalizedCoordinates(match.feature1.pos)
                        .hnormalized()};
        normMatch.feature2 =
            Feature{camera2.pixelToNormalizedCoordinates(match.feature2.pos)
                        .hnormalized()};

        normMatches.push_back(normMatch);
    }

    return normMatches;
}

int calcInliersVisibilityScore(const CameraMetaData& meta1,
                               const CameraMetaData& meta2,
                               const std::vector<Feature2D2D>& matches,
                               const std::vector<size_t>& inlierIndices)
{
    // If the image sizes are not available, do not make any assumptions
    // about what they might be. Default return the number of inliers.
    if (!meta1.imageSize.has_value() || !meta2.imageSize.has_value()) {
        return inlierIndices.size();
    }

    constexpr auto kPyramidLevels{6};

    VisibilityPyramid pyramid1{meta1.imageWidth(), meta1.imageHeight(),
                               kPyramidLevels};
    VisibilityPyramid pyramid2{meta2.imageWidth(), meta2.imageHeight(),
                               kPyramidLevels};
    for (const auto& i : inlierIndices) {
        const auto& match = matches[i];
        pyramid1.addPoint(match.feature1.pos);
        pyramid2.addPoint(match.feature2.pos);
    }

    return pyramid1.computeScore() + pyramid2.computeScore();
}

} // namespace

namespace EstimateViewPairInfo {

bool estimateCalibratedViewPair(const Options& opts,
                                const CameraMetaData& meta1,
                                const CameraMetaData& meta2,
                                const std::vector<Feature2D2D>& matches,
                                ViewPairInfo* info,
                                std::vector<size_t>* inlierIndices)
{
    const auto normMatches = normalizeMatches(meta1, meta2, matches);

    SacParameters sacParams;
    sacParams.rng = opts.rng;
    sacParams.failure_probability = 1. - opts.expected_ransac_confidence;
    sacParams.min_iterations = opts.min_ransac_iterations;
    sacParams.max_iterations = opts.max_ransac_iterations;
    sacParams.use_lo = opts.use_lo;
    sacParams.lo_start_iterations = opts.lo_start_iterations;
    sacParams.error_thresh = [&opts, &meta1, &meta2]() {
        // Scale the epipolar error threshold
        const auto maxEpipolarError1 =
            scaleEpipolarThreshold(opts.max_sampson_error_pixels,
                                   meta1.imageWidth(), meta1.imageHeight());
        const auto maxEpipolarError2 =
            scaleEpipolarThreshold(opts.max_sampson_error_pixels,
                                   meta2.imageWidth(), meta2.imageHeight());
        return maxEpipolarError1 * maxEpipolarError2 /
               (meta1.focal() * meta2.focal());
    }();
    sacParams.use_mle = opts.use_mle;

    RelativePose res;
    SacSummary summary;
    if (!EstimateRelativePose(sacParams, opts.ransac_type, normMatches, &res,
                              &summary)) {
        return false;
    }

    // Set the view pair info.
    info->rotation = [&res]() {
        const AngleAxisd aa{res.rotation};
        return aa.angle() * aa.axis();
    }();
    info->position = res.position;
    info->focalLength1 = meta1.focal();
    info->focalLength2 = meta2.focal();
    info->num_verified_matches = summary.inliers.size();
    info->visibility_score =
        calcInliersVisibilityScore(meta1, meta2, matches, *inlierIndices);

    *inlierIndices = summary.inliers;

    return true;
}

bool estimateUncalibratedViewPair(const Options& opts,
                                  const CameraMetaData& meta1,
                                  const CameraMetaData& meta2,
                                  const std::vector<Feature2D2D>& matches,
                                  ViewPairInfo* info,
                                  std::vector<size_t>* inlierIndices)
{
    const auto normMatches = normalizeMatches(meta1, meta2, matches);

    SacParameters sacParams;
    sacParams.rng = opts.rng;
    sacParams.failure_probability = 1. - opts.expected_ransac_confidence;
    sacParams.min_iterations = opts.min_ransac_iterations;
    sacParams.max_iterations = opts.max_ransac_iterations;
    sacParams.use_lo = opts.use_lo;
    sacParams.lo_start_iterations = opts.lo_start_iterations;
    sacParams.error_thresh = [&opts, &meta1, &meta2]() {
        // Scale the epipolar error threshold
        const auto maxEpipolarError1 =
            scaleEpipolarThreshold(opts.max_sampson_error_pixels,
                                   meta1.imageWidth(), meta1.imageHeight());
        const auto maxEpipolarError2 =
            scaleEpipolarThreshold(opts.max_sampson_error_pixels,
                                   meta2.imageWidth(), meta2.imageHeight());
        return maxEpipolarError1 * maxEpipolarError2;
    }();

    UncalibratedRelativePose res;
    SacSummary summary;
    if (!FindFundamental(normMatches, opts.min_focal_length,
                         opts.max_focal_length, opts.ransac_type, sacParams,
                         &res, &summary)) {
        return false;
    }

    // Set the view pair info.
    info->rotation = [&res]() {
        const AngleAxisd aa{res.rotation};
        return aa.angle() * aa.axis();
    }();
    info->position = res.position;
    info->focalLength1 = res.focalLength1;
    info->focalLength2 = res.focalLength2;
    info->num_verified_matches = summary.inliers.size();
    info->visibility_score =
        calcInliersVisibilityScore(meta1, meta2, matches, *inlierIndices);
    *inlierIndices = summary.inliers;

    return true;
}

bool Estimate(const Options& opts, const CameraMetaData& meta1,
              const CameraMetaData& meta2,
              const std::vector<Feature2D2D>& matches,
              ViewPairInfo* viewPairInfo, std::vector<size_t>* inlierIndices)
{
    CHECK(meta1.isValid() && meta2.isValid());
    CHECK_NOTNULL(viewPairInfo);
    CHECK_NOTNULL(inlierIndices)->clear();

    // Case 1: Both views are calibrated.
    if (meta1.calibrated() && meta2.calibrated()) {
        return estimateCalibratedViewPair(opts, meta1, meta2, matches,
                                          viewPairInfo, inlierIndices);
    }

    // Case 2: Only one of views are calibrated.
    if (meta1.calibrated() || meta2.calibrated()) {
        LOG(WARNING) << "Solving info of view pair of one calibrated view and "
                        "one uncalibrated view is not implemented yet. "
                        "Treating both views as uncalibrated instead.";
        return estimateUncalibratedViewPair(opts, meta1, meta2, matches,
                                            viewPairInfo, inlierIndices);
    }

    // Case 3: Both views are uncalibrated
    return estimateUncalibratedViewPair(opts, meta1, meta2, matches,
                                        viewPairInfo, inlierIndices);
}

} // namespace EstimateViewPairInfo

} // namespace tl
