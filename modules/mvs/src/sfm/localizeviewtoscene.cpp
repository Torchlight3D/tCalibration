#include "localizeviewtoscene.h"

#include <glog/logging.h>

#include <tMath/Ransac/RansacCreator>
#include <tMvs/Epipolar/Basics>
#include <tMvs/PnP/EstimateAbsolutePoseWithOrientation>
#include <tMvs/PnP/EstimateCalibratedAbsolutePose>
#include <tMvs/PnP/EstimateUncalibratedAbsolutePose>

namespace tl {

namespace {

bool DoesViewHaveKnownIntrinsics(const Scene& reconstruction,
                                 const ViewId view_id)
{
    const View* view = reconstruction.view(view_id);
    if (view->cameraMetaData().focalLength.has_value()) {
        return true;
    }

    // If the camera has shared intrinsics, return true if one of the shared
    // cameras has been estimated.
    const auto intrinsics_group_id = reconstruction.cameraId(view_id);
    const auto views_in_intrinsics_group =
        reconstruction.sharedCameraViewIds(intrinsics_group_id);
    for (const ViewId shared_intrinsics_view_id : views_in_intrinsics_group) {
        const View* view = reconstruction.view(view_id);
        if (view->estimated() && view_id != shared_intrinsics_view_id) {
            return true;
        }
    }
    return false;
}

void GetNormalized2D3DMatches(const Scene& reconstruction, const View& view,
                              std::vector<Feature2D3D>* matches)
{
    const Camera& camera = view.camera();
    const auto& tracks_in_view = view.trackIds();
    matches->reserve(tracks_in_view.size());
    for (const TrackId track_id : tracks_in_view) {
        const Track* track = reconstruction.track(track_id);
        // We only use 3D points that have been estimated.
        if (!track->estimated()) {
            continue;
        }

        Feature2D3D correspondence;
        const Feature& feature = *view.featureOf(track_id);
        // Simply shift the pixel to remove the effect of the principal point.
        correspondence.feature = feature.pos - camera.principalPoint();
        correspondence.world_point = track->position().hnormalized();
        matches->emplace_back(correspondence);
    }
}

void GetIntrinsicsNormalized2D3DMatches(const Scene& reconstruction,
                                        const View& view,
                                        std::vector<Feature2D3D>* matches)
{
    const Camera& camera = view.camera();
    const auto& tracks_in_view = view.trackIds();
    matches->reserve(tracks_in_view.size());
    for (const TrackId track_id : tracks_in_view) {
        const Track* track = reconstruction.track(track_id);
        // We only use 3D points that have been estimated.
        if (!track->estimated()) {
            continue;
        }

        Feature2D3D correspondence;
        const Feature& feature = *view.featureOf(track_id);
        // Remove the camera intrinsics from the feature.
        correspondence.feature =
            camera.pixelToNormalizedCoordinates(feature.pos).hnormalized();
        correspondence.world_point = track->position().hnormalized();
        matches->emplace_back(correspondence);
    }
}

bool EstimateCameraPose(bool known_intrinsics,
                        const LocalizeViewToReconstructionOptions& options,
                        const Scene& reconstruction, View* view,
                        SacSummary* summary)
{
    auto& camera = view->rCamera();

    // Gather all 2D-3D correspondences.
    std::vector<Feature2D3D> matches;
    if (known_intrinsics) {
        GetIntrinsicsNormalized2D3DMatches(reconstruction, *view, &matches);
    }
    else {
        GetNormalized2D3DMatches(reconstruction, *view, &matches);
    }

    // Exit early if there are not enough putative matches.
    if (matches.size() < options.min_num_inliers) {
        VLOG(2) << "Not enough 2D-3D correspondences to localize view "
                << view->name();
        return false;
    }

    // Set up the ransac parameters for absolute pose estimation.
    auto ransac_parameters = options.ransac_params;

    // Compute the reprojection error threshold scaled to account for the image
    // resolution.
    const double resolution_scaled_reprojection_error_threshold_pixels =
        scaleEpipolarThreshold(options.reprojection_error_threshold_pixels,
                               camera.imageWidth(), camera.imageHeight());

    // If we are assuming that the orientation is known then first try to use
    // the simplified camera positions solver.
    if (options.assume_known_orientation) {
        ransac_parameters.error_thresh =
            resolution_scaled_reprojection_error_threshold_pixels *
            resolution_scaled_reprojection_error_threshold_pixels /
            (camera.focalLength() * camera.focalLength());

        // Return true if position estimation is successful. Otherwise the
        // method will proceed to estimate the full pose.
        const Eigen::Vector3d camera_orientation =
            camera.orientationAsAngleAxis();
        Eigen::Vector3d camera_position;
        if (EstimateAbsolutePoseWithKnownOrientation(
                ransac_parameters, RansacType::RANSAC, camera_orientation,
                matches, &camera_position, summary) &&
            summary->inliers.size() > options.min_num_inliers) {
            camera.setPosition(camera_position);
            return true;
        }
        else {
            return false;
        }
    }

    // If calibrated, estimate the pose with P3P.
    bool success = false;
    if (known_intrinsics) {
        ransac_parameters.error_thresh =
            resolution_scaled_reprojection_error_threshold_pixels *
            resolution_scaled_reprojection_error_threshold_pixels /
            (camera.focalLength() * camera.focalLength());

        CalibratedAbsolutePose pose;
        if (EstimateCalibratedAbsolutePose(ransac_parameters,
                                           RansacType::RANSAC, options.pnp_type,
                                           matches, &pose, summary)) {
            camera.setOrientationFromRotationMatrix(pose.rotation);
            camera.setPosition(pose.position);
            return true;
        }
    }
    else {
        // If the focal length is not known, estimate the focal length and pose
        // together.
        ransac_parameters.error_thresh =
            resolution_scaled_reprojection_error_threshold_pixels *
            resolution_scaled_reprojection_error_threshold_pixels;

        UncalibratedAbsolutePose pose;
        if (EstimateUncalibratedAbsolutePose(ransac_parameters,
                                             RansacType::RANSAC, matches, &pose,
                                             summary)) {
            camera.setOrientationFromRotationMatrix(pose.rotation);
            camera.setPosition(pose.position);
            camera.setFocalLength(pose.focal_length);
            return true;
        }
    }

    return false;
}

} // namespace

bool LocalizeViewToReconstruction(
    const ViewId view_to_localize,
    const LocalizeViewToReconstructionOptions options, Scene* reconstruction,
    SacSummary* summary)
{
    CHECK_NOTNULL(reconstruction);
    CHECK_NOTNULL(summary);

    View* view = reconstruction->rView(view_to_localize);
    // We assume that the intrinsics are known if the orientation is known.
    const bool known_intrinsics =
        options.assume_known_orientation ||
        DoesViewHaveKnownIntrinsics(*reconstruction, view_to_localize);

    // If localization failed or did not produce a sufficient number of inliers
    // then return false.
    bool success = EstimateCameraPose(known_intrinsics, options,
                                      *reconstruction, view, summary);
    if (!success || summary->inliers.size() < options.min_num_inliers) {
        VLOG(2) << "Failed to localize view id " << view_to_localize
                << " with only " << summary->inliers.size() << " out of "
                << summary->num_input_data_points << " features as inliers.";
        return false;
    }

    // Bundle adjust the view if desired.
    view->setEstimated(true);
    if (options.bundle_adjust_view) {
        const auto summary = BundleAdjustView(options.ba_options,
                                              view_to_localize, reconstruction);
        success = summary.success;
    }

    VLOG(2) << "Estimated the camera pose for view " << view_to_localize
            << " with " << summary->inliers.size() << " inliers out of "
            << summary->num_input_data_points << " 2D-3D matches.";
    return success;
}

} // namespace tl
