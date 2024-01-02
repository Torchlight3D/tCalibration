#include "view.h"

#include <AxCore/ContainerUtils>

namespace thoht {

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

View::View(const std::string& name, double timestamp)
    : m_name(name),
      m_timestamp(timestamp),
      m_estimated(false),
      m_hasPosPrior(false)
{
    m_posPrior.setZero();
    m_posPriorSqrt.setIdentity();
}

const std::string& View::name() const { return m_name; }

const Camera& View::camera() const { return m_camera; }

Camera& View::rCamera() { return m_camera; }

void View::setEstimated(bool on) { m_estimated = on; }

bool View::estimated() const { return m_estimated; }

void View::setCameraMetaData(const CameraMetaData& meta)
{
    m_camMetaData = meta;
}

const CameraMetaData& View::cameraMetaData() const { return m_camMetaData; }

CameraMetaData& View::rCameraMetaData() { return m_camMetaData; }

size_t View::featureCount() const { return m_trackIdToFeature.size(); }

std::vector<TrackId> View::trackIds() const
{
    std::vector<TrackId> ids;
    ids.reserve(m_trackIdToFeature.size());
    for (const auto& [id, _] : m_trackIdToFeature) {
        ids.emplace_back(id);
    }
    return ids;
}

size_t View::trackCount() const { return m_trackIdToFeature.size(); }

const Feature* View::featureOf(int id) const
{
    return utils::FindOrNull(m_trackIdToFeature, id);
}

int View::trackIdOf(const Feature& feature) const
{
    if (const auto id = utils::FindOrNull(m_featureToTrackId, feature)) {
        return *id;
    }
    return kInvalidTrackId;
}

void View::addFeature(TrackId trackId, const Feature& feature)
{
    m_trackIdToFeature.insert({trackId, feature});
    m_featureToTrackId.insert({feature, trackId});
}

bool View::removeFeature(TrackId trackId)
{
    if (const auto feature = utils::FindOrNull(m_trackIdToFeature, trackId)) {
        return m_trackIdToFeature.erase(trackId) > 0 &&
               m_featureToTrackId.erase(*feature) > 0;
    }
    return false;
}

void View::setTimestamp(double timestamp) { m_timestamp = timestamp; }

double View::timestamp() const { return m_timestamp; }

Vector3d View::position() const { return m_camera.position(); }

Vector3d View::orientationAsEuler() const
{
    return m_camera.orientationAsEuler();
}

void View::setPositionPrior(const Vector3d& posPrior,
                            const Matrix3d& posPriorSqrt)
{
    m_posPrior = posPrior;
    m_posPriorSqrt = posPriorSqrt;
    m_hasPosPrior = true;
}

Vector3d View::positionPrior() const { return m_posPrior; }

Matrix3d View::positionPriorSqrt() const { return m_posPriorSqrt; }

bool View::hasPositionPrior() const { return m_hasPosPrior; }

void View::setTrackError(TrackId id, double error)
{
    if (const auto found = utils::FindOrNull(m_trackIdToFeature, id)) {
        // Replace or create
        m_trackIdToError[id] = error;
    }
}

bool View::trackError(TrackId id, double& error) const
{
    if (const auto found = utils::FindOrNull(m_trackIdToError, id)) {
        error = *found;
        return true;
    }

    return false;
}

double View::averageError() const
{
    double error{0.};
    for (const auto& [_, err] : m_trackIdToError) {
        error += err;
    }
    return error / trackCount();
}

void View::setTrackOffset(TrackId id, const Vector2d& offset)
{
    if (const auto found = utils::FindOrNull(m_trackIdToFeature, id)) {
        // Replace or create
        m_trackIdToOffset[id] = offset;
    }
}

bool View::trackOffset(TrackId id, Vector2d& offset) const
{
    if (const auto found = utils::FindOrNull(m_trackIdToOffset, id)) {
        offset = *found;
        return true;
    }

    return false;
}

} // namespace thoht
