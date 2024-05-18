#pragma once

#include <set>

#include <tMvs/Types>

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
    ViewId addView(const std::string& name, double timestamp);
    ViewId addView(const std::string& name, CameraId camId, double timestamp);
    bool removeView(ViewId id);
    void removeViews(CameraId id);

    const View* view(ViewId id) const;
    View* rView(ViewId id);
    int viewCount() const;
    int viewCount(CameraId id) const;

    ViewId viewIdFromName(const std::string& name) const;
    std::set<ViewId> viewIdsAtTime(double timestamp) const;
    std::vector<ViewId> viewIds() const;
    // Use left camera as reference
    void pairCamera(CameraId left, CameraId right);
    ViewId pairViewIdOf(ViewId id) const;

    std::vector<CameraId> cameraIds() const;
    int cameraCount() const;
    CameraId pairCameraIdOf(CameraId id) const;

    CameraId cameraId(ViewId id) const;
    const Camera* camera(CameraId id) const;
    std::vector<ViewId> sharedCameraViewIds(CameraId id) const;

    // Add empty track
    TrackId addTrack();
    void addTrack(TrackId id);
    TrackId addTrack(const std::vector<std::pair<ViewId, Feature>>& features);
    bool removeTrack(TrackId id);

    const Track* track(TrackId id) const;
    Track* rTrack(TrackId id);
    int trackCount() const;
    std::vector<TrackId> trackIds() const;

    // TODO: delete later
    std::unordered_map<TrackId, Track> tracks() const;
    std::unordered_map<ViewId, View> views() const;

    // Adds an observation between the track and the view to the scene. Returns
    // true upon successful insertion of the observation. If the track already
    // contains an observation to this view then false is returned. If the
    // view/track does not exist, or another failure is encountered then a
    // failure is thrown. sigma squared is the approximated measurement accuracy
    // of the image observation We set it to 1.0 pixel in the standard case.
    // However, if you know, that you can measure the image points more
    // accurately (e.g. aruco markers) you can set this value here
    bool addFeature(ViewId viewId, TrackId trackId, const Feature& feature);

    // Normalizes the reconstruction such that the "center" of the
    // reconstruction is moved to the origin and the reconstruction is scaled
    // such that the median distance of 3D points from the origin is 100.0. This
    // does not affect the reprojection error. A rotation is applied such that
    // the x-z plane is set to the dominating plane of the cameras.
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
    double calcViewReprojectionError(
        ViewId id, bool update = true,
        const Eigen::Vector3d* orientation = nullptr,
        const Eigen::Vector3d* translation = nullptr);

    void transform(const Eigen::Matrix3d& rotation,
                   const Eigen::Vector3d& translation, double scale);

    // Debug. TODO: Delete later
    const auto& timeToViewIds() const { return m_timestampToViewIds; }

private:
    std::unordered_map<std::string, ViewId> m_nameToViewId;
    std::map<double, std::set<ViewId>> m_timestampToViewIds;
    std::unordered_map<ViewId, View> m_viewIdToView;
    std::unordered_map<TrackId, Track> m_trackIdToTrack;
    std::unordered_map<ViewId, CameraId> m_viewIdToCamId;
    std::unordered_map<CameraId, std::set<ViewId>> m_camIdToViewIds;
    std::map<CameraId, CameraId> m_cameraPairs; // left to right, not used yet

    TrackId m_nextTrackId;
    ViewId m_nextViewId;
    CameraId m_nextCamId;
};

} // namespace tl
