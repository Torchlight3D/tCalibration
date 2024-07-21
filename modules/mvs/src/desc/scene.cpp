#include "scene.h"

#include <glog/logging.h>

#include <tCore/ContainerUtils>
#include <tMath/Eigen/Utils>

// TODO: I don't think Scene should take care of this
#include "../epipolar/triangulation.h"

namespace tl {

using Eigen::Matrix3d;
using Eigen::Matrix3Xd;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {

} // namespace

///-------- Scene starts from here
Scene::Scene() : m_nextTrackId(0), m_nextViewId(0), m_nextCamId(0) {}

Scene::~Scene() = default;

ViewId Scene::addView(const std::string& name, double timestamp, CameraId camId)
{
    if (name.empty()) {
        LOG(WARNING) << "Failed to add view to scene: "
                        "View name is empty.";
        return kInvalidViewId;
    }

    if (m_nameToViewId.contains(name)) {
        LOG(WARNING) << "Failed to add view to scene: "
                        "View name ("
                     << name << ") already exists.";
        return kInvalidViewId;
    }

    // FIXME: If exist, the existed ones should have same camId. It's impossible
    // to capture in more than one place at the same time.
    // if (m_timestampToViewIds.contains(timestamp)) {
    //     LOG(WARNING) << "Failed to add view to scene: View timestamp("
    //                  << timestamp << ") already exists.";
    //     return kInvalidViewId;
    // }

    if (camId == kInvalidCameraId) {
        camId = m_nextCamId++;
    }

    View view{name, timestamp};

    const auto& sharedViewIds = m_camIdToViewIds[camId];
    // If the camera already exists, set the internal
    // intrinsics to the same underlying intrinsics.
    if (!sharedViewIds.empty()) {
        // Any view is fine, they share the same intrinsics.
        const auto& camera =
            con::FindOrDie(m_viewIdToView, *sharedViewIds.begin()).camera();
        view.rCamera().setCameraIntrinsics(camera.cameraIntrinsics());
    }

    view.setTimestamp(timestamp);

    // Add the view relationships
    const auto viewId = m_nextViewId++;
    m_viewIdToView.emplace(viewId, view);
    m_nameToViewId.emplace(name, viewId);
    m_timestampToViewIds[timestamp].insert(viewId);
    // Add this view to the camera intrinsics group, and vice versa.
    m_viewIdToCamId.emplace(viewId, camId);
    m_camIdToViewIds[camId].emplace(viewId);

    return viewId;
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

std::vector<ViewId> Scene::estimatedViewIds() const
{
    std::vector<ViewId> ids;
    ids.reserve(m_viewIdToView.size());
    for (const auto& [id, view] : m_viewIdToView) {
        if (view.estimated()) {
            ids.push_back(id);
        }
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
    const auto id = m_nextTrackId;
    CHECK(!m_trackIdToTrack.contains(id)) << "Track already exists: Id " << id;

    Track track;
    m_trackIdToTrack.emplace(id, track);
    ++m_nextTrackId;
    return id;
}

void Scene::addTrack(TrackId id)
{
    CHECK(!m_trackIdToTrack.contains(id)) << "Track already exists: Id " << id;

    Track track;
    m_trackIdToTrack.emplace(id, track);
}

TrackId Scene::addTrack(const TrackObservation& trackObservation)
{
    if (trackObservation.size() < 2) {
        LOG(WARNING) << "A landmark must have at least 2 observations ("
                     << trackObservation.size()
                     << " were given). Cannot add landmark to the scene";
        return kInvalidTrackId;
    }

    const auto trackId = m_nextTrackId;
    CHECK(!m_trackIdToTrack.contains(trackId))
        << "The scene already contains a track with id: " << trackId;

    Track track;
    for (const auto& [viewId, feature] : trackObservation) {
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

std::vector<TrackId> Scene::trackIds() const
{
    std::vector<TrackId> ids;
    ids.reserve(m_trackIdToTrack.size());
    for (const auto& [id, _] : m_trackIdToTrack) {
        ids.push_back(id);
    }
    return ids;
}

std::vector<TrackId> Scene::estimatedTrackIds() const
{
    std::vector<TrackId> ids;
    ids.reserve(m_trackIdToTrack.size());
    for (const auto& [id, track] : m_trackIdToTrack) {
        if (track.estimated()) {
            ids.push_back(id);
        }
    }
    return ids;
}

std::vector<TrackId> Scene::trackIdsInViews(
    const std::vector<ViewId>& viewIds) const
{
    if (viewIds.empty()) {
        return {};
    }

    auto trackIdsInView = [this](ViewId id) -> std::vector<TrackId> {
        if (!m_viewIdToView.contains(id)) {
            return {};
        }

        return m_viewIdToView.at(id).trackIds();
    };

    auto commonTrackIds = trackIdsInView(viewIds.front());

    if (viewIds.size() == 1) {
        return commonTrackIds;
    }

    std::sort(commonTrackIds.begin(), commonTrackIds.end());

    std::vector<TrackId> buffer;
    for (const auto& viewId : viewIds) {
        if (commonTrackIds.empty()) {
            break;
        }

        buffer.clear();

        auto trackIds = trackIdsInView(viewId);
        std::sort(trackIds.begin(), trackIds.end());

        std::set_intersection(commonTrackIds.cbegin(), commonTrackIds.cend(),
                              trackIds.cbegin(), trackIds.cend(),
                              std::back_inserter(buffer));
        std::swap(commonTrackIds, buffer);
    }

    return commonTrackIds;
}

std::vector<Eigen::Vector3d> Scene::tracksInView(ViewId viewId) const
{
    if (!m_viewIdToView.contains(viewId)) {
        return {};
    }

    const auto trackIdsInView = m_viewIdToView.at(viewId).trackIds();

    std::vector<Vector3d> objPoints;
    objPoints.reserve(trackIdsInView.size());
    for (const auto& trackId : trackIdsInView) {
        if (m_trackIdToTrack.contains(trackId)) {
            objPoints.push_back(
                m_trackIdToTrack.at(trackId).position().hnormalized());
        }
    }

    return objPoints;
}

int Scene::trackCount() const
{
    return static_cast<int>(m_trackIdToTrack.size());
}

int Scene::setUnderconstrainedViewsUnestimated(int minTrackCount)
{
    auto underconstrainedViewCount{0};
    for (const auto& viewId : viewIds()) {
        auto* view = rView(viewId);
        if (!view || !view->estimated()) {
            continue;
        }

        auto estimatedTrackCount{0};
        for (const auto& trackId : view->trackIds()) {
            if (track(trackId)->estimated()) {
                ++estimatedTrackCount;
            }

            if (estimatedTrackCount >= minTrackCount) {
                break;
            }
        }

        if (estimatedTrackCount < minTrackCount) {
            view->setEstimated(false);
            ++underconstrainedViewCount;
        }
    }

    return underconstrainedViewCount;
}

int Scene::setUnderconstrainedTracksUnestimated(int minViewCount)
{
    auto underconstrainedTrackCount{0};
    for (const auto& trackId : trackIds()) {
        auto* track = rTrack(trackId);
        if (!track || !track->estimated()) {
            continue;
        }

        auto estimatedViewCount{0};
        for (const auto& viewId : track->viewIds()) {
            if (view(viewId)->estimated()) {
                ++estimatedViewCount;
            }

            if (estimatedViewCount >= minViewCount) {
                break;
            }
        }

        if (estimatedViewCount < minViewCount) {
            track->setEstimated(false);
            ++underconstrainedTrackCount;
        }
    }

    return underconstrainedTrackCount;
}

int Scene::setOutlierTracksUnestimated(const std::vector<TrackId>& trackIds,
                                       double maxRpe,
                                       double min_triangulation_angle_degrees)
{
    const double max_sq_reprojection_error = maxRpe * maxRpe;

    int num_estimated_tracks = 0;
    int num_bad_reprojections = 0;
    int num_insufficient_viewing_angles = 0;
    for (const auto& trackId :
         std::unordered_set<TrackId>(trackIds.cbegin(), trackIds.cend())) {
        auto* track = rTrack(trackId);
        if (!track->estimated()) {
            continue;
        }

        ++num_estimated_tracks;

        std::vector<Vector3d> rays;
        auto num_projections{0};
        auto mean_sq_reprojection_error{0.};
        for (const auto& viewId : track->viewIds()) {
            const auto* view = this->view(viewId);
            if (!view || !view->estimated()) {
                continue;
            }

            const auto& camera = view->camera();
            const Vector3d ray =
                track->position().hnormalized() - camera.position();
            rays.push_back(ray.normalized());

            // Check reprojection error.
            const auto* feature = view->featureOf(trackId);
            Vector2d projection;
            const double depth =
                camera.projectPoint(track->position(), projection);
            // Remove the feature if the reprojection is behind the camera.
            if (depth < 0.) {
                ++num_bad_reprojections;
                track->setEstimated(false);
                break;
            }

            mean_sq_reprojection_error +=
                (projection - (*feature).pos).squaredNorm();
            ++num_projections;
        }
        mean_sq_reprojection_error /= num_projections;

        if (track->estimated() &&
            mean_sq_reprojection_error > max_sq_reprojection_error) {
            ++num_bad_reprojections;
            track->setEstimated(false);
        }

        // The track will remain estimated if the reprojection errors were all
        // good. We then test that the track is properly constrained by having
        // at least two cameras view it with a sufficient viewing angle.
        if (track->estimated() && !SufficientTriangulationAngle(
                                      rays, min_triangulation_angle_degrees)) {
            ++num_insufficient_viewing_angles;
            track->setEstimated(false);
        }
    }

    LOG_IF(INFO,
           num_bad_reprojections > 0 || num_insufficient_viewing_angles > 0)
        << num_bad_reprojections
        << " points were removed because of bad reprojection errors. "
        << num_insufficient_viewing_angles
        << " points were removed because they had insufficient viewing angles "
           "and were poorly constrained.";

    return num_bad_reprojections + num_insufficient_viewing_angles;
}

void Scene::setTracksInViewsUnestimated(const std::vector<ViewId>& viewIds,
                                        const std::vector<TrackId>& exceptions)
{
    const std::unordered_set<TrackId> exceptions_set{exceptions.cbegin(),
                                                     exceptions.cend()};
    for (const auto viewId : viewIds) {
        const auto* view = this->view(viewId);
        if (!view || !view->estimated()) {
            continue;
        }

        for (const auto trackId : view->trackIds()) {
            if (auto* track = rTrack(trackId);
                track && !exceptions_set.contains(trackId)) {
                track->setEstimated(false);
            }
        }
    }
}

void Scene::resetEstimated()
{
    // Set tracks as unestimated.
    for (const auto& trackId : trackIds()) {
        if (auto* track = rTrack(trackId); track && track->estimated()) {
            track->setEstimated(false);
        }
    }

    // Set views as unestimated.
    for (const auto& viewId : viewIds()) {
        if (auto* view = rView(viewId); view && view->estimated()) {
            view->setEstimated(false);
        }
    }
}

void Scene::updateViewPoses(
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    const std::unordered_map<ViewId, Eigen::Vector3d>& positions)
{
    for (const auto& [viewId, position] : positions) {
        auto* view = rView(viewId);
        if (!view) {
            LOG(WARNING) << "Failed to update pose of View " << viewId
                         << ": "
                            "View does not exist in the scene.";
            continue;
        }

        if (view->estimated()) {
            LOG(WARNING) << "Failed to update pose of View " << viewId
                         << ": "
                            "View is already estimated.";
            continue;
        }

        const auto* orientation = con::FindOrNull(orientations, viewId);
        if (!orientation) {
            LOG(WARNING) << "Failed to update pose of View " << viewId
                         << ": "
                            "Missing estimated orientation.";
            continue;
        }

        view->rCamera().setPosition(position);
        view->rCamera().setOrientationFromAngleAxis(*orientation);
        view->setEstimated(true);
    }
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

    const auto& observedViewIds = track->viewIds();
    if (observedViewIds.contains(viewId)) {
        LOG(WARNING) << "Cannot add a new observation of track " << trackId
                     << " because the track is already observed by view "
                     << viewId;
        return false;
    }

    view->addFeature(trackId, feature);
    track->addView(viewId);
    return true;
}

void Scene::normalize()
{
    auto findMedianPoint = [](const std::vector<Eigen::Vector3d>& points) {
        const Eigen::Map<const Matrix3Xd> P(points[0].data(), 3, points.size());
        std::vector<double> x{P.row(0).data(),
                              P.row(0).data() + P.row(0).size()};
        std::vector<double> y{P.row(1).data(),
                              P.row(1).data() + P.row(1).size()};
        std::vector<double> z{P.row(2).data(),
                              P.row(2).data() + P.row(2).size()};
        const auto x_med = con::FindMedian(x);
        const auto y_med = con::FindMedian(y);
        const auto z_med = con::FindMedian(z);
        return Vector3d{x_med, y_med, z_med};
    };

    // First normalize the position so that the marginal median of the camera
    // positions is at the origin.
    std::vector<Vector3d> positions;
    const auto viewIds = this->viewIds();
    for (const auto& viewId : viewIds) {
        if (const auto* view = this->view(viewId); view && view->estimated()) {
            positions.push_back(view->camera().position());
        }
    }

    const auto t_med = findMedianPoint(positions);
    transform(Matrix3d::Identity(), -t_med);

    // Get the estimated track ids.
    const auto tempTrackIds = this->trackIds();
    std::vector<TrackId> trackIds;
    trackIds.reserve(tempTrackIds.size());
    for (const auto& id : tempTrackIds) {
        if (const auto track = this->track(id); track && track->estimated()) {
            trackIds.emplace_back(id);
        }
    }

    if (trackIds.empty()) {
        return;
    }

    // Compute the marginal median of the 3D points.
    std::vector<Vector3d> points;
    for (const auto& id : trackIds) {
        const auto track = this->track(id);
        points.push_back(track->position().hnormalized());
    }

    const auto pt_med = findMedianPoint(points);

    // Find the median absolute deviation of the points from the median.
    std::vector<double> distance_to_median;
    distance_to_median.reserve(trackIds.size());
    for (const auto& id : trackIds) {
        const auto track = this->track(id);
        const Vector3d point = track->position().hnormalized();
        distance_to_median.emplace_back((point - pt_med).lpNorm<1>());
    }

    // This will scale the scene so that the median absolute deviation of the
    // points is 100.
    const auto scale = 100. / con::FindMedian(distance_to_median);
    transform(Matrix3d::Identity(), Vector3d::Zero(), scale);

    // Most images are taken relatively upright with the x-direction of the
    // image parallel to the ground plane. We can solve for the transformation
    // that tries to best align the x-directions to the ground plane by finding
    // the null vector of the covariance matrix of per-camera x-directions.
    Matrix3d correlation = Matrix3d::Zero();
    for (const auto& viewId : viewIds) {
        if (const auto* view = this->view(viewId); view && view->estimated()) {
            const Vector3d x =
                view->camera().orientationAsRotationMatrix().transpose() *
                Vector3d::UnitX();
            correlation += x * x.transpose();
        }
    }

    // The up-direction is computed as the null vector of the covariance matrix.
    Eigen::JacobiSVD<Matrix3d> svd{correlation, Eigen::ComputeFullV};
    const Vector3d planeNormal = svd.matrixV().rightCols<1>();

    // We want the coordinate system to be such that the cameras lie on the x-z
    // plane with the y vector pointing up. Thus, the plane normal should be
    // equal to the positive y-direction.
    const Matrix3d rotation =
        Quaterniond::FromTwoVectors(planeNormal, Vector3d::UnitY())
            .toRotationMatrix();
    transform(rotation, Vector3d::Zero());
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
        // sub scene. We perform a set intersection to find these views.
        const auto& observedViewIds = track->viewIds();
        const auto commonViewIds =
            con::ContainerIntersection(subViewIds, observedViewIds);

        // Add the common views to the new track.
        for (const auto& viewId : commonViewIds) {
            newTrack.addView(viewId);
        }

        // Set the track in the sub scene.
        subScene->m_trackIdToTrack.emplace(trackId, newTrack);
    }
}

std::optional<double> Scene::calcViewReprojectionError(
    ViewId viewId, bool update, const Eigen::Vector3d* rvec,
    const Eigen::Vector3d* tvec)
{
    if (!m_viewIdToView.contains(viewId)) {
        return {};
    }

    auto* view = this->rView(viewId);

    std::unordered_map<TrackId, Vector2d> offsets;
    auto viewRpe{0.};
    for (const auto& trackId : view->trackIds()) {
        const auto* feature = view->featureOf(trackId);
        const auto* track = this->track(trackId);
        Vector2d pt;
        view->camera().projectPoint(track->position(), pt, rvec, tvec);
        const auto offset = (*feature).pos - pt;

        viewRpe += offset.norm();
        offsets.insert({trackId, offset});
    }
    viewRpe /= view->trackCount();

    if (update) {
        for (const auto& [trackId, offset] : offsets) {
            view->setTrackOffset(trackId, offset);
        }
    }

    return std::make_optional(viewRpe);
}

void Scene::transform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                      double scale)
{
    for (const auto& viewId : viewIds()) {
        if (auto* view = rView(viewId); view->estimated()) {
            view->rCamera().transform(R, t, scale);
        }
    }

    for (const auto& trackId : trackIds()) {
        if (auto* track = rTrack(trackId); track->estimated()) {
            Vector3d point = track->position().hnormalized();
            math::transformPoint(R, t, scale, point);
            track->rPosition() = point.homogeneous();
        }
    }
}

} // namespace tl
