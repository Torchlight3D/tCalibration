#include "trackbuilder.h"

#include <tCore/ContainerUtils>

#include "../desc/scene.h"

namespace tl {

TrackBuilder::TrackBuilder(size_t minTrackCount, size_t maxTrackCount)
    : connected_components_(
          std::make_unique<ConnectedComponents<TrackObservationId>>(
              maxTrackCount)),
      _obsId(0),
      _minTrackCount(minTrackCount)
{
}

TrackBuilder::~TrackBuilder() = default;

void TrackBuilder::addFeatureMatch(ViewId viewId1, const Feature& feature1,
                                   ViewId viewId2, const Feature& feature2,
                                   int* cornerId)
{
    CHECK_NE(viewId1, viewId2)
        << "Cannot add 2 features from the same image as a correspondence for "
           "track generation.";

    const auto obs1 = std::make_pair(viewId1, feature1);
    const auto obs2 = std::make_pair(viewId2, feature2);

    const auto obsId1 = addObservation(obs1);
    const auto obsId2 = addObservation(obs2);

    connected_components_->AddEdge(obsId1, obsId2);

    // TEST:
    if (cornerId) {
        _obsIdToCornerId.emplace(obsId1, *cornerId);
        _obsIdToCornerId.emplace(obsId1, *cornerId);
    }
}

void TrackBuilder::buildTracks(Scene* scene)
{
    CHECK_NOTNULL(scene);

    // Build a temp reverse mapping from ObservationId to Observation.
    std::unordered_map<TrackObservationId, const TrackObservation*> idToObs;
    idToObs.reserve(_obsToId.size());
    for (const auto& [obs, id] : _obsToId) {
        con::InsertOrDie(&idToObs, id, &obs);
    }

    // Extract all connected components.
    std::unordered_map<TrackObservationId,
                       std::unordered_set<TrackObservationId>>
        components;
    connected_components_->Extract(&components);

    // Each connected component is a track. Add all tracks to the scene.
    auto numPoorFeatures{0};
    auto numInvalidFeatures{0};
    for (const auto& [obsId, neighborObsIds] : components) {
        if (neighborObsIds.size() < _minTrackCount) {
            ++numPoorFeatures;
            continue;
        }

        // Add all features in the connected component to the track.
        Scene::TrackObservation observations;
        observations.reserve(neighborObsIds.size());
        std::unordered_set<ViewId> viewIds;
        for (const auto& neighborObsId : neighborObsIds) {
            const auto& [viewId, feat] =
                *con::FindOrDie(idToObs, neighborObsId);

            // Do not add the feature if the track already contains a feature
            // from the same image.
            if (!con::InsertIfNotPresent(&viewIds, viewId)) {
                ++numInvalidFeatures;
                continue;
            }

            observations.emplace(viewId, feat);
        }

        const auto trackId = scene->addTrack(observations);
        if (trackId == kInvalidTrackId) {
            LOG(WARNING) << "Failed to add tracks.";
            continue;
        }

        if (_obsIdToCornerId.contains(obsId)) {
            _cornerIdToTrackIds[_obsIdToCornerId.at(obsId)].emplace(trackId);
        }
    }

    LOG(INFO)
        << scene->trackCount()
        << " tracks were created. "
           "\n"
        << numInvalidFeatures
        << " features are dropped because they have confusing Id. "
           "\n"
        << numPoorFeatures
        << " features are dropped because they donot have enough observations.";
}

void TrackBuilder::buildTracksIncremental(Scene* scene)
{
    CHECK_NOTNULL(scene);

    // Build a temp reverse mapping from ObservationId to Observation.
    std::unordered_map<TrackObservationId, const TrackObservation*> idToObs;
    idToObs.reserve(_obsToId.size());
    for (const auto& [obs, obsId] : _obsToId) {
        con::InsertOrDie(&idToObs, obsId, &obs);
    }

    // Extract all connected components.
    std::unordered_map<TrackObservationId,
                       std::unordered_set<TrackObservationId>>
        components;
    connected_components_->Extract(&components);

    // Each connected component is a track. Add all tracks to the scene.
    auto numPoorFeatures{0};
    auto numInvalidFeatures{0};
    for (const auto& [obsId, neighborObsIds] : components) {
        if (neighborObsIds.size() < _minTrackCount) {
            ++numPoorFeatures;
            continue;
        }

        // Connected component already has a track in the scene
        if (const auto found = _obsIdToTrackId.find(obsId);
            found != _obsIdToTrackId.cend()) {
            const auto& trackId = found->second;
            const auto& observingViewIds = scene->track(trackId)->viewIds();
            for (const auto& neighborObsId : neighborObsIds) {
                const auto& [viewId, feat] =
                    *con::FindOrDie(idToObs, neighborObsId);
                if (!observingViewIds.contains(viewId)) {
                    scene->addFeature(viewId, trackId, feat);
                }
            }
        }
        // Initialize a new track
        else {
            // Add all features in the connected component to the track.
            Scene::TrackObservation observations;
            observations.reserve(neighborObsIds.size());
            std::unordered_set<ViewId> viewIds;
            for (const auto& neighborObsId : neighborObsIds) {
                const auto& [viewId, feat] =
                    *con::FindOrDie(idToObs, neighborObsId);

                // Do not add the feature if the track already contains a
                // feature from the same image.
                if (!con::InsertIfNotPresent(&viewIds, viewId)) {
                    ++numInvalidFeatures;
                    continue;
                }

                observations.emplace(viewId, feat);
            }

            const auto newTrackId = scene->addTrack(observations);
            CHECK_NE(newTrackId, kInvalidTrackId) << "Could not build tracks.";
            _obsIdToTrackId[obsId] = newTrackId;
        }
    }

    LOG(INFO)
        << scene->trackCount()
        << " tracks were created. "
           "\n"
        << numInvalidFeatures
        << " features are dropped because they have confusing Id. "
           "\n"
        << numPoorFeatures
        << " features are dropped because they donot have enough observations.";
}

uint64_t TrackBuilder::addObservation(const TrackObservation& observation)
{
    if (const auto* obsId = con::FindOrNull(_obsToId, observation)) {
        return *obsId;
    }

    const auto obsId = _obsId++;
    con::InsertOrDieNoPrint(&_obsToId, observation, obsId);

    return obsId;
}

} // namespace tl
