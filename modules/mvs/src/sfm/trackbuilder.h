#pragma once

#include <map>
#include <memory>
#include <set>

#include <tCore/HashUtils>

#include <tMvs/Feature>
#include <tMvs/Types>
#include <tMvs/Graph/ConnectedComponents>

namespace tl {

class Scene;

// Brief:
// Build tracks from feature correspondences across multiple images. Tracks are
// created with the connected components algorithm and have a maximum allowable
// size. If there are multiple features from one image in a track, we do not do
// any intelligent selection and just arbitrarily choose a feature to drop so
// that the tracks are consistent.
class TrackBuilder
{
    using TrackObservation = std::pair<ViewId, Feature>;
    using TrackObservationId = uint64_t;

public:
    TrackBuilder(size_t minTrackCount, size_t maxTrackCount);
    ~TrackBuilder();

    void addFeatureMatch(ViewId viewId1, const Feature& feature1,
                         ViewId viewId2, const Feature& feature2,
                         int* cornerId = nullptr);

    // Generates all tracks and adds them to the scene.
    void buildTracks(Scene* scene);

    // This function can be called incrementally for sequential scenes
    // It checks if connected components already exist
    void buildTracksIncremental(Scene* scene);

    /// Debug
    const auto& cornerIdToTrackId() const { return _cornerIdToTrackIds; }

private:
    TrackObservationId addObservation(const TrackObservation& observation);

private:
    std::unordered_map<TrackObservation, TrackObservationId> _obsToId;
    std::unique_ptr<ConnectedComponents<TrackObservationId>>
        connected_components_;
    // for incremental mapping
    std::unordered_map<TrackObservationId, TrackId> _obsIdToTrackId;
    TrackObservationId _obsId;
    const size_t _minTrackCount;

    // TEST
    std::unordered_map<TrackObservationId, int> _obsIdToCornerId;
    std::map<int, std::set<TrackId>> _cornerIdToTrackIds;
};

} // namespace tl
