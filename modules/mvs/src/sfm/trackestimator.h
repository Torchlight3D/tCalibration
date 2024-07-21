#pragma once

#include <atomic>
#include <mutex>
#include <unordered_set>
#include <vector>

#include <tMvs/BA/BundleAdjustment>
#include <tMvs/Epipolar/Triangulation>

namespace tl {

class Scene;

// Estimates the 3D point of a track by using all estimated views to compute a
// (potentially nonminimal) triangulation of track. The the angle between all
// views and the triangulated point must be greater than the minimum
// triangulation error. The track estimation is successful if all views have a
// reprojection error less than the specified max reprojection error.
// Estimates all unestimated tracks in the reconstruction.
class TrackEstimator
{
public:
    struct Options
    {
        // Number of threads for multithreading.
        int num_threads = 1;

        // Maximum reprojection error for successful triangulation.
        double max_acceptable_reprojection_error_pixels = 5.0;

        // Minimum triangulation angle between two views required for
        // triangulation. For N-View triangulation we require that at least one
        // pair of views has this an angle this large.
        double min_triangulation_angle_degrees = 3.0;

        // Perform bundle adjustment on the track as soon as a position is
        // estimated.
        bool bundle_adjustment = true;
        BundleAdjustment::Options ba_options;

        // For thread-level parallelism, it is better to estimate a small fixed
        // number of tracks per thread worker instead of 1 track per worker.
        // This number controls how many points are estimated per worker.
        int multithreaded_step_size = 100;

        // Triangulation method
        TriangulationMethodType triangulation_method =
            TriangulationMethodType::MIDPOINT;
    };

    TrackEstimator(const Options& options, Scene* scene);

    struct Summary
    {
        // Number of estimated tracks that were input.
        int input_num_estimated_tracks = 0;

        // Number of triangulation attempts made.
        int num_triangulation_attempts = 0;

        // TrackId of the newly estimated tracks. This set does not include
        // tracks that were input as estimated.
        std::unordered_set<TrackId> estimated_tracks;
    };

    // Estimate all unestimated tracks.
    Summary EstimateAllTracks();

    // Estimate specific tracks.
    Summary EstimateTracks(const std::unordered_set<TrackId>& trackIds);

private:
    void EstimateTrackSet(TrackId start, TrackId stop);
    bool EstimateTrack(TrackId id);

private:
    const Options options_;
    Scene* _scene; // Not owned, const
    std::vector<TrackId> tracks_to_estimate_;

    // A mutex lock for setting the summary
    TrackEstimator::Summary summary_;
    std::mutex summary_mutex_;

    std::atomic_int num_bad_angles_, num_failed_triangulations_,
        num_bad_reprojections_;
};

} // namespace tl
