#include "landmark.h"

namespace thoht {

Track::Track() : m_invDepth(0.), m_refViewId(kInvalidViewId), m_estimated(false)
{
    m_pos.setZero();
    reference_bearing_.setZero();
}

void Track::setPosition(const Eigen::Vector4d& point) { m_pos = point; }

const Eigen::Vector4d& Track::position() const { return m_pos; }

Eigen::Vector4d& Track::rPosition() { return m_pos; }

void Track::setInverseDepth(double val) { m_invDepth = val; }

const double* Track::inverseDepth() const { return &m_invDepth; }

double* Track::rInverseDepth() { return &m_invDepth; }

void Track::setColor(const Eigen::Vector3i& color) { m_color = color; }

const Eigen::Vector3i& Track::color() const { return m_color; }

Eigen::Vector3i& Track::rColor() { return m_color; }

void Track::setReferenceBearing(const Eigen::Vector3d& bearing)
{
    reference_bearing_ = bearing;
}

const Eigen::Vector3d& Track::referenceBearing() const
{
    return reference_bearing_;
}

void Track::setReferenceDescriptor(const Eigen::VectorXf& descriptor)
{
    reference_descriptor_ = descriptor;
}

const Eigen::VectorXf& Track::referenceDescriptor() const
{
    return reference_descriptor_;
}

void Track::addView(ViewId id)
{
    m_viewIds.insert(id);
    if (m_refViewId == kInvalidViewId) {
        m_refViewId = id;
    }
}

bool Track::removeView(ViewId id)
{
    bool removed = m_viewIds.erase(id) > 0;
    if (removed && m_viewIds.size() >= 1) {
        m_refViewId = *m_viewIds.begin();
    }
    else {
        m_refViewId = kInvalidViewId;
    }
    return removed;
}

int Track::viewCount() const { return static_cast<int>(m_viewIds.size()); }

bool Track::noView() const { return m_viewIds.empty(); }

ViewId Track::referenceViewId() const { return m_refViewId; }

const std::unordered_set<ViewId>& Track::viewIds() const { return m_viewIds; }

void Track::setEstimated(bool est) { m_estimated = est; }

bool Track::estimated() const { return m_estimated; }

void Track::setError(double err) { m_error = err; }

double Track::error() const { return m_error; }

} // namespace thoht
