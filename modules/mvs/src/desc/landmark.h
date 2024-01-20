#pragma once

#include <unordered_set>
#include <Eigen/Core>

#include "mvs_types.h"

namespace tl {

class Track
{
public:
    Track();
    ~Track() = default;

    /// Properties
    void setPosition(const Eigen::Vector4d& position);
    const Eigen::Vector4d& position() const;
    Eigen::Vector4d& rPosition();

    void setInverseDepth(double value);
    const double* inverseDepth() const;
    double* rInverseDepth();

    void setColor(const Eigen::Vector3i& color);
    const Eigen::Vector3i& color() const;
    Eigen::Vector3i& rColor();

    void setReferenceBearing(const Eigen::Vector3d& bearing);
    const Eigen::Vector3d& referenceBearing() const;

    void setReferenceDescriptor(const Eigen::VectorXf& descriptor);
    const Eigen::VectorXf& referenceDescriptor() const;

    void setEstimated(bool est);
    bool estimated() const;

    void setError(double err);
    double error() const;

    /// Relationship
    void addView(ViewId viewId);
    bool removeView(ViewId viewId);

    int viewCount() const;
    bool noView() const;

    ViewId referenceViewId() const;
    const std::unordered_set<ViewId>& viewIds() const;

private:
    // TODO: Not only ViewId, FeatureId as well.
    std::unordered_set<ViewId> m_viewIds;
    Eigen::Vector4d m_pos;
    Eigen::Vector3d reference_bearing_;    // not used for now
    Eigen::VectorXf reference_descriptor_; // not used for now
    Eigen::Vector3i m_color;
    double m_invDepth;
    double m_error;
    ViewId m_refViewId;
    bool m_estimated;
};

} // namespace tl
