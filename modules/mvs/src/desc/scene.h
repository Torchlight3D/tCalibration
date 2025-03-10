﻿#pragma once

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "landmark.h"
#include "view.h"

namespace tl {

struct Feature;

class Scene
{
public:
    using Ptr = std::shared_ptr<Scene>;
    using ConstPtr = std::shared_ptr<const Scene>;

    Scene();
    ~Scene();

    // Default use a new underlying camera
    ViewId addView(const std::string& name, double timestamp,
                   CameraId id = kInvalidCameraId);
    bool removeView(ViewId id);
    void removeViews(CameraId id);

    const View* view(ViewId id) const;
    View* rView(ViewId id);
    int viewCount() const;
    int viewCount(CameraId id) const;

    ViewId viewIdFromName(const std::string& name) const;
    std::set<ViewId> viewIdsAtTime(double timestamp) const;
    std::vector<ViewId> viewIds() const;
    std::vector<ViewId> estimatedViewIds() const;
    std::vector<ViewId> sharedCameraViewIds(CameraId id) const;

    // Use left camera as reference
    void pairCamera(CameraId left, CameraId right);
    ViewId pairViewIdOf(ViewId id) const;

    std::vector<CameraId> cameraIds() const;
    int cameraCount() const;
    CameraId pairCameraIdOf(CameraId id) const;

    CameraId cameraId(ViewId id) const;
    const Camera* camera(CameraId id) const;

    // Add empty track
    TrackId addTrack();
    void addTrack(TrackId id);
    using TrackObservation = std::unordered_map<ViewId, Feature>;
    TrackId addTrack(const TrackObservation& observation);
    bool removeTrack(TrackId id);

    const Track* track(TrackId id) const;
    Track* rTrack(TrackId id);
    std::vector<TrackId> trackIds() const;
    std::vector<TrackId> estimatedTrackIds() const;
    std::vector<TrackId> trackIdsInViews(const std::vector<ViewId>& ids) const;
    std::vector<Eigen::Vector3d> tracksInView(ViewId id) const;
    int trackCount() const;

    int setUnderconstrainedViewsUnestimated(int minTrackCount = 3);
    int setUnderconstrainedTracksUnestimated(int minViewCount = 2);
    int setOutlierTracksUnestimated(const std::vector<TrackId>& trackIds,
                                    double maxRpe,
                                    double min_triangulation_angle_degrees);
    inline auto setOutlierTracksUnestimated(
        double maxRpe, double min_triangulation_angle_degrees)
    {
        return setOutlierTracksUnestimated(trackIds(), maxRpe,
                                           min_triangulation_angle_degrees);
    }

    void setTracksInViewsUnestimated(const std::vector<ViewId>& viewIds,
                                     const std::vector<TrackId>& exceptions);
    void resetEstimated();

    void updateViewPoses(
        const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
        const std::unordered_map<ViewId, Eigen::Vector3d>& positions);

    // Adds an observation between the track and the view to the scene. Returns
    // true upon successful insertion of the observation. If the track already
    // contains an observation to this view then false is returned. If the
    // view/track does not exist, or another failure is encountered then a
    // failure is thrown. sigma squared is the approximated measurement accuracy
    // of the image observation We set it to 1.0 pixel in the standard case.
    // However, if you know, that you can measure the image points more
    // accurately (e.g. aruco markers) you can set this value here
    bool addFeature(ViewId viewId, TrackId trackId, const Feature& feature);

    // Normalizes the scene such that the "center" of the scene is moved to the
    // origin and the scene is scaled such that the median distance of 3D points
    // from the origin is 100.0. This does not affect the reprojection error. A
    // rotation is applied such that the x-z plane is set to the dominating
    // plane of the cameras.
    //
    // NOTE: This implementation is inspired by the BAL problem normalization in
    // Ceres Solver.
    void normalize();

    // Obtain a sub-scene which only contains the specified views and
    // corresponding tracks observed by those views. All views and tracks
    // maintain the same IDs as in the original scene.
    void extractSubScene(const std::unordered_set<ViewId>& subViewIds,
                         Scene* subScene) const;

    // Return average RPE of observed points in a view
    // TODO: Put all RPE calculation into one place
    std::optional<double> calcViewReprojectionError(
        ViewId id, bool update = true,
        const Eigen::Vector3d* orientation = nullptr,
        const Eigen::Vector3d* translation = nullptr);

    void transform(const Eigen::Matrix3d& rotation,
                   const Eigen::Vector3d& translation, double scale = 1.);

    // Debug. TODO: Delete later
    const auto& timeToViewIds() const { return m_timestampToViewIds; }

private:
    std::unordered_map<std::string, ViewId> m_nameToViewId;
    // FIXME: Not safe to use floating point as map key
    std::map<double, std::set<ViewId>> m_timestampToViewIds;
    std::unordered_map<ViewId, View> m_viewIdToView;
    std::unordered_map<ViewId, CameraId> m_viewIdToCamId;
    std::unordered_map<CameraId, std::set<ViewId>> m_camIdToViewIds;
    std::map<CameraId, CameraId> m_cameraPairs; // left to right, not used yet
    std::unordered_map<TrackId, Track> m_trackIdToTrack;

    TrackId m_nextTrackId;
    ViewId m_nextViewId;
    CameraId m_nextCamId;
};

} // namespace tl
