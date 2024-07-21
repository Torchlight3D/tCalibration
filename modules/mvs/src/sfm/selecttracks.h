#pragma once

#include <vector>

#include <tMvs/Types>

namespace tl {

class Scene;

// Brief:
// The efficiency of large scale bundle adjustment can be dramatically increased
// by choosing only a subset of 3d points to optimize, as the 3d points tend to
// have increasing scene redundancy. If the points are chosen in a way that
// properly constrains the nonlinear optimization, similar results in quality
// may be observed compared to optimizing all tracks.
//
// The 3d points are preferred such that they fit the following criteria:
//    a. High confidence (i.e. low reprojection error).
//    b. Long tracks.
//    c. Good detected feature spatial coverage in each image.
//    d. Each view observes at least K optimized tracks.
//
// Tracks in each image are first hashed into spatial bins with an image grid
// where each image grid cell is the provided width. Within each grid cell, the
// tracks are ordered based on their track length, then by mean reprojection
// error. The track length is truncated to be no longer than max track observed
// count so that among long tracks, the ones with low reprojection error are
// chosen for bundle adjustment.

namespace SelectGoodTracks {

struct Options
{
    int maxTrackObservedCount = 10;

    // In pixel
    int imageCellSize = 100;

    int minTrackPerView = 100;

    Options() {}
};

// [Overload]: Default use all the estimated views in scene
std::vector<TrackId> Select(const Options& opts, const Scene& scene);

std::vector<TrackId> Select(const Options opts, const Scene& scene,
                            const std::vector<ViewId>& viewIds);

} // namespace SelectGoodTracks

} // namespace tl
