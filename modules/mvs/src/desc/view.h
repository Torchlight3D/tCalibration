#pragma once

#include <string>
#include <Eigen/Core>

#include <AxCamera/Camera>
#include "feature.h"
#include "mvs_types.h"

namespace thoht {

struct Feature;

class View
{
public:
    explicit View(const std::string& name = {}, double timestamp = 0.);
    ~View() = default;

    const std::string& name() const;

    void setTimestamp(double timestamp);
    double timestamp() const;

    const Camera& camera() const;
    Camera& rCamera();

    void setCameraMetaData(const CameraMetaData& meta);
    const CameraMetaData& cameraMetaData() const;
    CameraMetaData& rCameraMetaData();

    void addFeature(TrackId id, const Feature& feature);
    bool removeFeature(TrackId id);
    size_t featureCount() const;

    const Feature* featureOf(TrackId id) const;
    TrackId trackIdOf(const Feature& feature) const;

    std::vector<TrackId> trackIds() const;
    size_t trackCount() const;

    Eigen::Vector3d position() const;
    Eigen::Vector3d orientationAsEuler() const;

    void setPositionPrior(const Eigen::Vector3d& positionPrior,
                          const Eigen::Matrix3d& positionPriorSqrt);
    Eigen::Vector3d positionPrior() const;
    Eigen::Matrix3d positionPriorSqrt() const;
    bool hasPositionPrior() const;

    void setTrackError(TrackId id, double error);
    bool trackError(TrackId id, double& error) const;
    double averageError() const;
    void setTrackOffset(TrackId id, const Eigen::Vector2d& offset);
    bool trackOffset(TrackId id, Eigen::Vector2d& offset) const;

    void setEstimated(bool est);
    bool estimated() const;

private:
    Camera m_camera;
    CameraMetaData m_camMetaData;
    std::string m_name;
    double m_timestamp;
    bool m_estimated;
    std::unordered_map<TrackId, Feature> m_trackIdToFeature;
    std::unordered_map<Feature, TrackId> m_featureToTrackId;

    // For analysis and visualization
    std::unordered_map<TrackId, double> m_trackIdToError;
    std::unordered_map<TrackId, Eigen::Vector2d> m_trackIdToOffset;

    // Prior on an absolute position (e.g. GPS), not used yet
    Eigen::Vector3d m_posPrior;
    Eigen::Matrix3d m_posPriorSqrt;
    bool m_hasPosPrior;
};

} // namespace thoht
