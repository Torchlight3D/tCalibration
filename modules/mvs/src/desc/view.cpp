#include "view.h"

#include <tCore/ContainerUtils>

namespace tl {

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

void View::setTimestamp(double timestamp) { m_timestamp = timestamp; }

double View::timestamp() const { return m_timestamp; }

const Camera& View::camera() const { return m_camera; }

Camera& View::rCamera() { return m_camera; }

void View::setCameraMetaData(const CameraMetaData& meta)
{
    m_camMetaData = meta;
}

const CameraMetaData& View::cameraMetaData() const { return m_camMetaData; }

CameraMetaData& View::rCameraMetaData() { return m_camMetaData; }

void View::addFeature(TrackId id, const Feature& feature)
{
    m_trackIdToFeature.insert({id, feature});
    m_featureToTrackId.insert({feature, id});
}

bool View::removeFeature(TrackId id)
{
    if (const auto feature = con::FindOrNull(m_trackIdToFeature, id)) {
        return m_trackIdToFeature.erase(id) > 0 &&
               m_featureToTrackId.erase(*feature) > 0;
    }
    return false;
}

const Feature* View::featureOf(TrackId id) const
{
    return con::FindOrNull(m_trackIdToFeature, id);
}

std::vector<Eigen::Vector2d> View::features() const
{
    std::vector<Vector2d> imgPoints;
    imgPoints.reserve(m_trackIdToFeature.size());
    for (const auto& [_, feat] : m_trackIdToFeature) {
        imgPoints.push_back(feat.pos);
    }
    return imgPoints;
}

size_t View::featureCount() const { return m_trackIdToFeature.size(); }

TrackId View::trackIdOf(const Feature& feature) const
{
    if (const auto id = con::FindOrNull(m_featureToTrackId, feature)) {
        return *id;
    }
    return kInvalidTrackId;
}

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

bool View::empty() const { return m_trackIdToFeature.empty(); }

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

void View::setTrackOffset(TrackId id, const Vector2d& offset)
{
    if (const auto found = con::FindOrNull(m_trackIdToFeature, id)) {
        m_trackIdToOffset[id] = offset;
    }
}

std::optional<Eigen::Vector2d> View::trackOffset(TrackId id) const
{
    if (const auto found = con::FindOrNull(m_trackIdToOffset, id)) {
        return std::make_optional(*found);
    }

    return {};
}

std::optional<double> View::trackError(TrackId id) const
{
    if (const auto offset = trackOffset(id); offset.has_value()) {
        return std::make_optional(offset.value().norm());
    }

    return {};
}

double View::trackErrorMean() const
{
    auto error{0.};
    for (const auto& [_, offset] : m_trackIdToOffset) {
        error += offset.norm();
    }
    return error / trackCount();
}

void View::setEstimated(bool on) { m_estimated = on; }

bool View::estimated() const { return m_estimated; }

} // namespace tl
