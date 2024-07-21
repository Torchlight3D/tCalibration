#include "selecttracks.h"

#include <tCore/ContainerUtils>
#include <tMath/Eigen/Utils>
#include <tMvs/Scene>

namespace tl {

using Eigen::Vector2d;
using Eigen::Vector2i;

namespace SelectGoodTracks {

namespace internal {

/// Track statistics

// [Observing count, RPE mean squared]
using TrackStatistics = std::pair<int, double>;

TrackStatistics calcTrackStatistics(const Scene& scene, TrackId trackId,
                                    int maxObservedCount)
{
    const auto* track = scene.track(trackId);
    const auto& observingViewIds = track->viewIds();

    auto rpe_ms{0.};
    auto observingCount{0};
    for (const auto& viewId : observingViewIds) {
        if (const auto* view = scene.view(viewId); view && view->estimated()) {
            const auto& point = view->featureOf(trackId)->pos;
            Vector2d reproj;
            view->camera().projectPoint(track->position(), reproj);
            rpe_ms += (reproj - point).squaredNorm();
            ++observingCount;
        }
    }
    rpe_ms /= observingCount;

    return {std::min(observingCount, maxObservedCount), rpe_ms};
}

std::unordered_map<TrackId, TrackStatistics> calcTrackStatisticsInViews(
    const Scene& scene, const std::vector<ViewId>& viewIds,
    int maxObservingCount)
{
    std::unordered_map<TrackId, TrackStatistics> trackStatistics;
    for (const auto& viewId : viewIds) {
        const auto* view = scene.view(viewId);
        for (const auto& trackId : view->trackIds()) {
            const auto* track = scene.track(trackId);
            if (track && track->estimated() &&
                !trackStatistics.contains(trackId)) {
                trackStatistics.emplace(
                    trackId,
                    calcTrackStatistics(scene, trackId, maxObservingCount));
            }
        }
    }

    return trackStatistics;
}

/// Image grid

// Select tracks from the image to ensure good spatial coverage of the image. To
// do this, we first bin the tracks into grid cells in an image grid. Then
// within each cell we find the best ranked track and add it to the list of
// tracks to optimize.

using ImageCellIndex = Vector2i;
using ImageCellElement = std::pair<TrackId, TrackStatistics>;
using ImageCellElements = std::vector<ImageCellElement>;
using ImageGrid = std::unordered_map<ImageCellIndex, ImageCellElements>;

std::unordered_set<TrackId> selectBestTracksFromImageGrid(
    const Scene& scene, const View& view, int cellSize,
    const std::unordered_map<TrackId, TrackStatistics>& trackStatistics)
{
    ImageGrid grid;
    for (const auto& trackId : view.trackIds()) {
        const auto* track = scene.track(trackId);
        if (!track || !track->estimated()) {
            continue;
        }

        const auto* feature = view.featureOf(trackId);
        const auto& trackStatistic =
            con::FindOrDieNoPrint(trackStatistics, trackId);
        const Vector2i index = (feature->pos / cellSize).cast<int>();

        grid[index].emplace_back(trackId, trackStatistic);
    }

    // Select the best feature from each grid cell and add it to the tracks to
    // optimize.
    std::unordered_set<TrackId> bestTrackIds;
    for (const auto& [_, cell] : grid) {
        // Sorts the grid cell elements by the track statistics, which will sort
        // first by the (truncated) track length, then by the mean reprojection
        // error. Order the features in each cell by track length first, then
        // mean reprojection error.
        const auto& element =
            *std::min_element(cell.cbegin(), cell.cend(),
                              [](const auto& element1, const auto& element2) {
                                  return element1.second < element2.second;
                              });

        bestTrackIds.emplace(element.first);
    }

    return bestTrackIds;
}

// Selects the top ranked tracks that have not already been chosen until the
// view observes the minimum number of optimized tracks.
void selectBestTracksInView(
    const Scene& scene,
    const std::unordered_map<TrackId, TrackStatistics>& trackStatistics,
    const View& view, int minSelectTrackPerView,
    std::unordered_set<TrackId>* goodTrackIds)
{
    int selectedTrackCount = 0;
    int estimatedTrackCount = 0;
    std::vector<ImageCellElement> candidates;
    for (const auto& trackId : view.trackIds()) {
        const auto* track = scene.track(trackId);
        if (!track || !track->estimated()) {
            continue;
        }

        ++estimatedTrackCount;

        if (goodTrackIds->contains(trackId)) {
            ++selectedTrackCount;

            if (selectedTrackCount >= minSelectTrackPerView) {
                return;
            }
        }
        else {
            candidates.emplace_back(
                trackId, con::FindOrDieNoPrint(trackStatistics, trackId));
        }
    }

    // We only reach this point if the number of optimized tracks is less than
    // the minimum. If that is the case then we add the top candidate features
    // until the minimum number of features observed is met.
    if (selectedTrackCount != estimatedTrackCount) {
        const int more = std::min(minSelectTrackPerView - selectedTrackCount,
                                  estimatedTrackCount - selectedTrackCount);
        std::partial_sort(candidates.begin(), candidates.begin() + more,
                          candidates.end());
        for (int i = 0; i < more; i++) {
            goodTrackIds->emplace(candidates[i].first);
        }
    }
}

} // namespace internal

std::vector<TrackId> Select(const Options& opts, const Scene& scene)
{
    const auto viewIds = scene.estimatedViewIds();
    return Select(opts, scene, viewIds);
}

std::vector<TrackId> Select(const Options opts, const Scene& scene,
                            const std::vector<ViewId>& viewIds)
{
    const auto trackStatistics = internal::calcTrackStatisticsInViews(
        scene, viewIds, opts.maxTrackObservedCount);

    // For each image, divide the image into a grid and choose the highest
    // quality tracks from each grid cell. This encourages good spatial coverage
    // of tracks within each image.
    std::unordered_set<TrackId> goodTrackIds;
    for (const auto& viewId : viewIds) {
        const auto* view = scene.view(viewId);

        // Select the best tracks from each grid cell in the image and add them
        // to the container of tracks to be optimized.
        const auto trackIds = internal::selectBestTracksFromImageGrid(
            scene, *view, opts.imageCellSize, trackStatistics);
        goodTrackIds.insert(trackIds.cbegin(), trackIds.cend());
    }

    // To this point, we have only added features that have as full spatial
    // coverage as possible within each image but we have not ensured that each
    // image is constrainted by at least K features. So, we cycle through all
    // views again and add the top M tracks that have not already been added.
    for (const auto& viewId : viewIds) {
        const auto* view = scene.view(viewId);

        // If this view is not constrained by enough optimized tracks, add the
        // top ranked features until there are enough tracks constraining the
        // view.
        internal::selectBestTracksInView(scene, trackStatistics, *view,
                                         opts.minTrackPerView, &goodTrackIds);
    }

    return {goodTrackIds.cbegin(), goodTrackIds.cend()};
}

} // namespace SelectGoodTracks

} // namespace tl
