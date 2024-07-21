#pragma once

#include <memory>
#include <vector>

#include <tMath/Types>

namespace tl {

class RandomNumberGenerator;
struct CameraMetaData;
struct Feature2D2D;
struct ViewPairInfo;

namespace EstimateViewPairInfo {

struct Options
{
    // Ransac parameters
    std::shared_ptr<RandomNumberGenerator> rng;
    RansacType ransac_type = RansacType::RANSAC;
    double expected_ransac_confidence = 0.9999;
    int min_ransac_iterations = 10;
    int max_ransac_iterations = 1000;
    bool use_mle = true;
    bool use_lo = false;
    int lo_start_iterations = 10;

    // Maximum epipolar (sampson) error in pixels for matches to be inliers.
    // NOTE: This threshold is with respect to the image of 1024 pixels wide.
    // If the image dimensions are larger or smaller than this value then the
    // threshold will be scaled accordingly.
    double max_sampson_error_pixels = 6.;

    // Initial focal length guess if available
    double min_focal_length = 1.;
    double max_focal_length = std::numeric_limits<double>::max();
};

// Brief:
// Estimates two view info for the given view pair from the correspondences. The
// correspondences should be in pixel coordinates (the method will perform
// normalization w.r.t focal length as necessary).
//
// There are three cases for estimating two view infos:
//   1. Both views are calibrated. In this case, the essential matrix is
//      estimated then decomposed to compute the two view info.
//   2. Both views are uncalibrated. The fundamental matrix is estimated and
//      decomposed to compute the two view info. The quality of the focal length
//      is not always great with fundamental matrix decomposition.
//   3. One view is calibrated and one view is uncalibrated. This case is
//      currently unsupported, and case 2 will be used instead.
bool Estimate(const Options& options, const CameraMetaData& meta1,
              const CameraMetaData& meta2,
              const std::vector<Feature2D2D>& matches, ViewPairInfo* info,
              std::vector<size_t>* inlierIndices);

} // namespace EstimateViewPairInfo

} // namespace tl
