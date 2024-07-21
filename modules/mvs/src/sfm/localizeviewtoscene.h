#pragma once

#include <tMath/Ransac/SampleConsensus>
#include <tMvs/Types>
#include <tMvs/BA/BundleAdjustment>

namespace tl {

class Scene;

namespace LocalizeViewToView {

struct Options
{
    // The reprojection error threshold that determines whether a 2D-3D
    // correspondence is an inlier during localization.
    //
    // NOTE: This threshold is with respect to an image that is 1024 pixels
    // wide. If the image dimensions are larger or smaller than this value then
    // the threshold will be appropriately scaled.
    double reprojection_error_threshold_pixels = 4.0;

    // If true, a simplified pose solver will be used to estimate the camera
    // position given the known orientation. If that solver is not successful,
    // then standard P3P is used.
    bool assume_known_orientation = false;

    // The RANSAC parameters used for robust estimation in the localization
    // algorithms.
    SacParameters ransac_params;

    // The view will be bundle adjusted (while all tracks are held constant) if
    // this is set to true.
    bool bundle_adjust_view = true;
    BundleAdjustment::Options ba_options;

    // The minimum number of inliers found from RANSAC in order to be considered
    // successful localization.
    int min_num_inliers = 30;

    // PNPType is the type of PnP algorithm to use for calibrated localization.
    PnPType pnp_type = PnPType::DLS;
};

bool Run(ViewId view_to_localize, const Options options, Scene* reconstruction,
         SacSummary* summary);

} // namespace LocalizeViewToView

// The reprojection_error_threshold_pixels is the threshold (measured in pixels)
// that determines inliers and outliers during RANSAC. This value will override
// the error thresh set in the RansacParameters.
struct LocalizeViewToReconstructionOptions
{
    // The reprojection error threshold that determines whether a 2D-3D
    // correspondence is an inlier during localization.
    //
    // NOTE: This threshold is with respect to an image that is 1024 pixels
    // wide. If the image dimensions are larger or smaller than this value then
    // the threshold will be appropriately scaled.
    double reprojection_error_threshold_pixels = 4.0;

    // If true, a simplified pose solver will be used to estimate the camera
    // position given the known orientation. If that solver is not successful,
    // then standard P3P is used.
    bool assume_known_orientation = false;

    // The RANSAC parameters used for robust estimation in the localization
    // algorithms.
    SacParameters ransac_params;

    // The view will be bundle adjusted (while all tracks are held constant) if
    // this is set to true.
    bool bundle_adjust_view = true;
    BundleAdjustment::Options ba_options;

    // The minimum number of inliers found from RANSAC in order to be considered
    // successful localization.
    int min_num_inliers = 30;

    // PNPType is the type of PnP algorithm to use for calibrated localization.
    PnPType pnp_type = PnPType::DLS;
};

// Localizes a view to the reconstruction using 2D-3D correspondences to
// estimate the absolute camera pose. If the focal length is known then the P3P
// algorithm is used, otherwise P4Pf is used to additionally recover the focal
// length.
bool LocalizeViewToReconstruction(
    const ViewId view_to_localize,
    const LocalizeViewToReconstructionOptions options, Scene* reconstruction,
    SacSummary* summary);

} // namespace tl
