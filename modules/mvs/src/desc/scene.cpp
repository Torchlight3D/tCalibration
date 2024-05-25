#include "scene.h"

#include <glog/logging.h>
#include <tCore/ContainerUtils>

namespace tl {

namespace {

bool checkDuplicateViewsInTrack(
    const std::vector<std::pair<int, Feature>>& tracks)
{
    std::vector<ViewId> viewIds;
    viewIds.reserve(tracks.size());
    for (const auto& [id, _] : tracks) {
        viewIds.push_back(id);
    }

    std::sort(viewIds.begin(), viewIds.end());
    return std::adjacent_find(viewIds.begin(), viewIds.end()) != viewIds.end();
}

void transformPoint(const Eigen::Matrix3d& rmat, const Eigen::Vector3d& tvec,
                    double scale, Eigen::Vector3d& point)
{
    point = scale * rmat * point + tvec;
}

void transformCamera(const Eigen::Matrix3d& rmat, const Eigen::Vector3d& tvec,
                     double scale, Camera& camera)
{
    const auto camOrientation = camera.orientationAsRotationMatrix();
    camera.setOrientationFromRotationMatrix(camOrientation * rmat.transpose());

    Eigen::Vector3d cam_pos = camera.position();
    transformPoint(rmat, tvec, scale, cam_pos);
    camera.setPosition(cam_pos);
}

} // namespace

///-------- Scene starts from here
Scene::Scene() : m_nextTrackId(0), m_nextViewId(0), m_nextCamId(0) {}

Scene::~Scene() = default;

ViewId Scene::addView(const std::string& name, double timestamp)
{
    const auto viewId = addView(name, m_nextCamId, timestamp);
    ++m_nextCamId;
    return viewId;
}

ViewId Scene::addView(const std::string& name, CameraId camId, double timestamp)
{
    if (name.empty()) {
        LOG(WARNING) << "Failed to add view to scene: View name is empty.";
        return kInvalidViewId;
    }

    if (m_nameToViewId.contains(name)) {
        LOG(WARNING) << "Failed to add view to scene: View name(" << name
                     << ") already exists.";
        return kInvalidViewId;
    }

    // FIXME: If exist, the existed ones should have same camId. It's impossible
    // to capture in more than one place at the same time.
    //    if (ContainsKey(m_timestampToViewIds, timestamp)) {
    //        LOG(WARNING) << "Failed to add view to scene: View timestamp("
    //                     << timestamp << ") already exists.";
    //        return kInvalidViewId;
    //    }

    View view{name, timestamp};

    const auto& sharedViewIds = m_camIdToViewIds[camId];
    // If the camera already exists, set the internal
    // intrinsics to the same underlying intrinsics.
    if (!sharedViewIds.empty()) {
        // Any view is fine, they share the same intrinsics.
        const auto viewId = *sharedViewIds.begin();
        const auto& camera = con::FindOrDie(m_viewIdToView, viewId).camera();
        view.rCamera().setCameraIntrinsics(camera.cameraIntrinsics());
    }

    view.setTimestamp(timestamp);

    // Add the view relationships
    m_viewIdToView.emplace(m_nextViewId, view);
    m_nameToViewId.emplace(name, m_nextViewId);
    m_timestampToViewIds[timestamp].insert(m_nextViewId);
    // Add this view to the camera intrinsics group, and vice versa.
    m_viewIdToCamId.emplace(m_nextViewId, camId);
    m_camIdToViewIds[camId].emplace(m_nextViewId);

    ++m_nextViewId;
    return m_nextViewId - 1;
}

bool Scene::removeView(ViewId viewId)
{
    auto* view = con::FindOrNull(m_viewIdToView, viewId);
    if (!view) {
        LOG(WARNING) << "Failed to remove the view from the scene: "
                        "The view does not exist.";
        return false;
    }

    // Remove related tracks.
    const auto observedTrackIds = view->trackIds();
    for (const auto& trackId : observedTrackIds) {
        auto* track = rTrack(trackId);
        if (!track) {
            LOG(WARNING) << "Failed to remove the view from the landmark: "
                            "The landmark does not exist.";
            return false;
        }

        if (!track->removeView(viewId)) {
            LOG(WARNING) << "Could not remove the view from the track";
            return false;
        }

        if (track->noView()) {
            removeTrack(trackId);
        }
    }

    // Remove related infos.
    m_nameToViewId.erase(view->name());
    m_timestampToViewIds[view->timestamp()].erase(viewId);
    if (m_timestampToViewIds[view->timestamp()].empty()) {
        m_timestampToViewIds.erase(view->timestamp());
    }

    // Remove the view from the camera groups.
    const auto camId = cameraId(viewId);
    m_viewIdToCamId.erase(viewId);
    auto& sharedCameraViewIds = con::FindOrDie(m_camIdToViewIds, camId);
    sharedCameraViewIds.erase(viewId);
    if (sharedCameraViewIds.empty()) {
        m_camIdToViewIds.erase(camId);
    }

    // TODO: Remove view pair
    //

    // Remove the view.
    m_viewIdToView.erase(viewId);
    return true;
}

void Scene::removeViews(CameraId camId)
{
    const auto viewIds = sharedCameraViewIds(camId);
    for (const auto& viewId : viewIds) {
        removeView(viewId);
    }
}

const View* Scene::view(ViewId id) const
{
    return con::FindOrNull(m_viewIdToView, id);
}

View* Scene::rView(ViewId id) { return con::FindOrNull(m_viewIdToView, id); }

int Scene::viewCount() const { return static_cast<int>(m_viewIdToView.size()); }

int Scene::viewCount(CameraId id) const
{
    const auto sharedCamViewIds = sharedCameraViewIds(id);
    return sharedCamViewIds.size();
}

ViewId Scene::viewIdFromName(const std::string& name) const
{
    return con::FindWithDefault(m_nameToViewId, name, kInvalidViewId);
}

std::set<ViewId> Scene::viewIdsAtTime(double t) const
{
    return con::FindWithDefault(m_timestampToViewIds, t, std::set<ViewId>{});
}

std::vector<ViewId> Scene::viewIds() const
{
    std::vector<ViewId> ids;
    ids.reserve(m_viewIdToView.size());
    for (const auto& [id, _] : m_viewIdToView) {
        ids.push_back(id);
    }
    return ids;
}

CameraId Scene::cameraId(ViewId id) const
{
    return con::FindWithDefault(m_viewIdToCamId, id, kInvalidCameraId);
}

const Camera* Scene::camera(CameraId camId) const
{
    const auto sharedCamViewIds = sharedCameraViewIds(camId);
    if (sharedCamViewIds.empty()) {
        return nullptr;
    }

    return &view((*sharedCamViewIds.begin()))->camera();
}

std::vector<CameraId> Scene::cameraIds() const
{
    std::vector<CameraId> ids;
    ids.reserve(m_camIdToViewIds.size());
    for (const auto& [id, _] : m_camIdToViewIds) {
        ids.emplace_back(id);
    }
    return ids;
}

std::vector<ViewId> Scene::sharedCameraViewIds(CameraId id) const
{
    const auto viewIds =
        con::FindWithDefault(m_camIdToViewIds, id, std::set<ViewId>());
    if (viewIds.empty()) {
        return {};
    }

    return {viewIds.begin(), viewIds.end()};
}

int Scene::cameraCount() const
{
    return static_cast<int>(m_camIdToViewIds.size());
}

void Scene::pairCamera(CameraId leftCamId, CameraId rightCamId)
{
    m_cameraPairs.insert({leftCamId, rightCamId});
}

CameraId Scene::pairCameraIdOf(CameraId id) const
{
    return con::FindWithDefault(m_cameraPairs, id, kInvalidCameraId);
}

TrackId Scene::addTrack()
{
    const auto trackId = m_nextTrackId;
    CHECK(!m_trackIdToTrack.contains(trackId))
        << "Track already exists: Id " << trackId;

    Track track;
    m_trackIdToTrack.emplace(trackId, track);
    ++m_nextTrackId;
    return trackId;
}

void Scene::addTrack(TrackId id)
{
    CHECK(!m_trackIdToTrack.contains(id)) << "Track already exists: Id " << id;

    Track track;
    m_trackIdToTrack.emplace(id, track);
}

bool Scene::addFeature(ViewId viewId, TrackId trackId, const Feature& feature)
{
    CHECK(m_viewIdToView.contains(viewId))
        << "View does not exist. AddObservation may only be used to add "
           "observations to an existing view.";
    CHECK(m_trackIdToTrack.contains(trackId))
        << "Track does not exist. AddObservation may only be used to add "
           "observations to an existing track.";

    auto* view = con::FindOrNull(m_viewIdToView, viewId);
    auto* track = con::FindOrNull(m_trackIdToTrack, trackId);
    if (view->featureOf(trackId)) {
        LOG(WARNING) << "Cannot add a new observation of track " << trackId
                     << " because the view already contains an observation of "
                        "the track.";
        return false;
    }

    const std::unordered_set<int>& views_observing_track = track->viewIds();
    if (views_observing_track.contains(viewId)) {
        LOG(WARNING) << "Cannot add a new observation of track " << trackId
                     << " because the track is already observed by view "
                     << viewId;
        return false;
    }

    view->addFeature(trackId, feature);
    track->addView(viewId);
    return true;
}

int Scene::addTrack(
    const std::vector<std::pair<ViewId, Feature>>& viewIdToFeature)
{
    if (viewIdToFeature.size() < 2) {
        LOG(WARNING) << "A landmark must have at least 2 observations ("
                     << viewIdToFeature.size()
                     << " were given). Cannot add landmark to the scene";
        return kInvalidTrackId;
    }

    if (checkDuplicateViewsInTrack(viewIdToFeature)) {
        LOG(WARNING)
            << "Cannot add a track that contains the same view twice to "
               "the reconstruction.";
        return kInvalidTrackId;
    }

    const auto trackId = m_nextTrackId;
    CHECK(!m_trackIdToTrack.contains(trackId))
        << "The reconstruction already contains a track with id: " << trackId;

    Track track;
    for (const auto& [viewId, feature] : viewIdToFeature) {
        // Make sure the view exists in the model.
        CHECK(m_viewIdToView.contains(viewId))
            << "Cannot add a track with containing an observation in view id "
            << viewId << " because the view does not exist.";

        // Add view to track.
        track.addView(viewId);

        // Add track to view.
        auto* view = rView(viewId);
        view->addFeature(trackId, feature);
    }

    m_trackIdToTrack.emplace(trackId, track);
    ++m_nextTrackId;
    return trackId;
}

bool Scene::removeTrack(TrackId trackId)
{
    auto* track = con::FindOrNull(m_trackIdToTrack, trackId);
    if (!track) {
        LOG(WARNING) << "Failed to remove track: TrackId " << trackId
                     << " does not exist.";
        return false;
    }

    for (const auto& viewId : track->viewIds()) {
        auto* view = con::FindOrNull(m_viewIdToView, viewId);
        if (!view) {
            LOG(WARNING) << "Failed to remove view: ViewId " << viewId
                         << " does not exist.";
            return false;
        }

        if (!view->removeFeature(trackId)) {
            LOG(WARNING) << "Failed to remove track: TrackId " << trackId
                         << " is not observed by view " << viewId << ".";
            return false;
        }
    }

    m_trackIdToTrack.erase(trackId);
    return true;
}

const Track* Scene::track(TrackId id) const
{
    return con::FindOrNull(m_trackIdToTrack, id);
}

Track* Scene::rTrack(TrackId id)
{
    return con::FindOrNull(m_trackIdToTrack, id);
}

int Scene::trackCount() const
{
    return static_cast<int>(m_trackIdToTrack.size());
}

std::unordered_map<TrackId, Track> Scene::tracks() const
{
    return m_trackIdToTrack;
}

std::unordered_map<ViewId, View> Scene::views() const { return m_viewIdToView; }

std::vector<TrackId> Scene::trackIds() const
{
    std::vector<TrackId> ids;
    ids.reserve(m_trackIdToTrack.size());
    for (const auto& [id, _] : m_trackIdToTrack) {
        ids.push_back(id);
    }
    return ids;
}

void Scene::normalize()
{
    // First normalize the position so that the marginal median of the camera
    // positions is at the origin.
    std::vector<std::vector<double>> camera_positions(3);
    Eigen::Vector3d median_camera_position;
    const auto& viewIds = this->viewIds();
    for (const auto& view_id : viewIds) {
        const View* view = this->view(view_id);
        if (!view || !view->estimated()) {
            continue;
        }

        const Eigen::Vector3d point = view->camera().position();
        camera_positions[0].push_back(point[0]);
        camera_positions[1].push_back(point[1]);
        camera_positions[2].push_back(point[2]);
    }
    median_camera_position(0) = con::FindMedian(camera_positions[0]);
    median_camera_position(1) = con::FindMedian(camera_positions[1]);
    median_camera_position(2) = con::FindMedian(camera_positions[2]);
    transform(Eigen::Matrix3d::Identity(), -median_camera_position, 1.0);

    // Get the estimated track ids.
    const auto& tempTrackIds = this->trackIds();
    std::vector<TrackId> trackIds;
    trackIds.reserve(tempTrackIds.size());
    for (const auto& track_id : tempTrackIds) {
        const auto track = this->track(track_id);
        if (!track || !track->estimated()) {
            continue;
        }
        trackIds.emplace_back(track_id);
    }

    if (trackIds.empty()) {
        return;
    }

    // Compute the marginal median of the 3D points.
    std::vector<std::vector<double>> points(3);
    Eigen::Vector3d median;
    for (const auto& id : trackIds) {
        const auto track = this->track(id);
        const Eigen::Vector3d point = track->position().hnormalized();
        points[0].push_back(point[0]);
        points[1].push_back(point[1]);
        points[2].push_back(point[2]);
    }
    median(0) = con::FindMedian(points[0]);
    median(1) = con::FindMedian(points[1]);
    median(2) = con::FindMedian(points[2]);

    // Find the median absolute deviation of the points from the median.
    std::vector<double> distance_to_median;
    distance_to_median.reserve(trackIds.size());
    for (const auto& id : trackIds) {
        const auto track = this->track(id);
        const Eigen::Vector3d point = track->position().hnormalized();
        distance_to_median.emplace_back((point - median).lpNorm<1>());
    }

    // This will scale the reconstruction so that the median absolute deviation
    // of the points is 100.
    const double scale = 100.0 / con::FindMedian(distance_to_median);
    transform(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), scale);

    // Most images are taken relatively upright with the x-direction of the
    // image parallel to the ground plane. We can solve for the transformation
    // that tries to best align the x-directions to the ground plane by finding
    // the null vector of the covariance matrix of per-camera x-directions.
    Eigen::Matrix3d correlation = Eigen::Matrix3d::Zero();
    for (const auto& viewId : viewIds) {
        const auto* view = this->view(viewId);
        if (!view || !view->estimated()) {
            continue;
        }

        const auto& camera = view->camera();
        const Eigen::Vector3d x =
            camera.orientationAsRotationMatrix().transpose() *
            Eigen::Vector3d{1.0, 0.0, 0.0};
        correlation += x * x.transpose();
    }

    // The up-direction is computed as the null vector of the covariance matrix.
    Eigen::JacobiSVD<Eigen::Matrix3d> svd{correlation, Eigen::ComputeFullV};
    const Eigen::Vector3d plane_normal = svd.matrixV().rightCols<1>();

    // We want the coordinate system to be such that the cameras lie on the x-z
    // plane with the y vector pointing up. Thus, the plane normal should be
    // equal to the positive y-direction.
    Eigen::Matrix3d rotation = Eigen::Quaterniond::FromTwoVectors(
                                   plane_normal, Eigen::Vector3d(0, 1, 0))
                                   .toRotationMatrix();
    transform(rotation, Eigen::Vector3d::Zero(), 1.0);
}

void Scene::extractSubScene(const std::unordered_set<ViewId>& subViewIds,
                            Scene* subScene) const
{
    CHECK_NOTNULL(subScene);

    // Copy the "next" ids.
    subScene->m_nextTrackId = m_nextTrackId;
    subScene->m_nextViewId = m_nextViewId;
    subScene->m_nextCamId = m_nextCamId;

    // Copy the view information. Also store the tracks in each view so that we
    // may easily retreive them below.
    subScene->m_viewIdToView.reserve(subViewIds.size());
    subScene->m_nameToViewId.reserve(subViewIds.size());
    //    subScene->m_timestampToViewId.reserve(subViewIds.size());
    std::unordered_set<TrackId> allTrackIds;
    for (const auto& viewId : subViewIds) {
        const auto* view = con::FindOrNull(m_viewIdToView, viewId);
        if (!view) {
            continue;
        }

        // Set the view information.
        subScene->m_viewIdToView[viewId] = *view;
        subScene->m_nameToViewId[view->name()] = viewId;
        subScene->m_timestampToViewIds[view->timestamp()].insert(viewId);
        // Set the intrinsics group id information.
        const auto& camId = con::FindOrDie(m_viewIdToCamId, viewId);
        subScene->m_viewIdToCamId[viewId] = camId;
        subScene->m_camIdToViewIds[camId].emplace(viewId);

        // Add the tracks from this view to our track container.
        const auto& trackIds = view->trackIds();
        allTrackIds.insert(trackIds.begin(), trackIds.end());
    }

    // Copy the tracks.
    subScene->m_trackIdToTrack.reserve(allTrackIds.size());
    for (const auto& trackId : allTrackIds) {
        const auto* track = con::FindOrNull(m_trackIdToTrack, trackId);
        if (!track) {
            continue;
        }

        // Create the new track by copying the values of the old track.
        Track newTrack;
        newTrack.setEstimated(track->estimated());
        newTrack.rPosition() = track->position();

        // The new track should only contain observations from views in the
        // subreconstruction. We perform a set intersection to find these views.
        const auto& observingViewIds = track->viewIds();
        const auto commonViewIds =
            con::ContainerIntersection(subViewIds, observingViewIds);

        // Add the common views to the new track.
        for (const auto& viewId : commonViewIds) {
            newTrack.addView(viewId);
        }

        // Set the track in the subreconstruction.
        subScene->m_trackIdToTrack.emplace(trackId, newTrack);
    }
}

double Scene::calcViewReprojectionError(ViewId viewId, bool update,
                                        const Eigen::Vector3d* rvec,
                                        const Eigen::Vector3d* tvec)
{
    auto* view = this->rView(viewId);

    struct LandmarkError
    {
        Eigen::Vector2d offset;
        double reprojectError;
    };

    std::unordered_map<TrackId, LandmarkError> landmarkErrors;

    double viewRPE{0.};
    for (const auto& trackId : view->trackIds()) {
        const auto* feature = view->featureOf(trackId);
        const auto* track = this->track(trackId);
        Eigen::Vector2d pt;
        view->camera().projectPoint(track->position(), pt, rvec, tvec);
        const auto offset = (*feature).pos - pt;
        const auto projectError = offset.norm();
        viewRPE += projectError;

        landmarkErrors.insert(
            {trackId, {.offset = offset, .reprojectError = projectError}});
    }
    viewRPE /= view->trackCount();

    // TODO: Useless? Now used by analysis and visualization.
    if (update) {
        for (const auto& [trackId, landmarkError] : landmarkErrors) {
            view->setTrackError(trackId, landmarkError.reprojectError);
            view->setTrackOffset(trackId, landmarkError.offset);
            auto* track = this->rTrack(trackId);
            track->setError(landmarkError.reprojectError);
        }
    }

    return viewRPE;
}

void Scene::transform(const Eigen::Matrix3d& rmat, const Eigen::Vector3d& tvec,
                      double scale)
{
    for (const auto& viewId : viewIds()) {
        auto* view = rView(viewId);
        if (view->estimated()) {
            transformCamera(rmat, tvec, scale, view->rCamera());
        }
    }

    for (const auto& trackId : trackIds()) {
        auto* track = rTrack(trackId);
        if (track->estimated()) {
            Eigen::Vector3d point = track->position().hnormalized();
            transformPoint(rmat, tvec, scale, point);
            track->rPosition() = point.homogeneous();
        }
    }
}

} // namespace tl
