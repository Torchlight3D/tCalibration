#pragma once

#include "quaternion.h"
#include "vec.h"

namespace viewer {
class Frame;
class Camera;

class Constraint
{
public:
    virtual ~Constraint() {}

    virtual void constrainTranslation(Vec &translation, Frame *const frame)
    {
        Q_UNUSED(translation);
        Q_UNUSED(frame);
    }
    virtual void constrainRotation(Quaternion &rotation, Frame *const frame)
    {
        Q_UNUSED(rotation);
        Q_UNUSED(frame);
    }
};

class AxisPlaneConstraint : public Constraint
{
public:
    AxisPlaneConstraint();
    virtual ~AxisPlaneConstraint() {}

    enum class Type
    {
        Free,
        Axis,
        Plane,
        Forbidden
    };
    void setTranslationConstraintType(Type type)
    {
        m_transConstraintType = type;
    };
    Type translationConstraintType() const { return m_transConstraintType; };
    void setTranslationConstraintDirection(const Vec &direction);
    Vec translationConstraintDirection() const { return m_transConstraintDir; };
    void setTranslationConstraint(Type type, const Vec &direction);

    void constrainTranslation(Vec &translation, Frame *const frame) override
    {
        Q_UNUSED(translation);
        Q_UNUSED(frame);
    };

    void setRotationConstraintType(Type type);
    Type rotationConstraintType() const { return m_rotConstraintType; };
    void setRotationConstraintDirection(const Vec &direction);
    Vec rotationConstraintDirection() const { return m_rotConstraintDir; };
    void setRotationConstraint(Type type, const Vec &direction);

    void constrainRotation(Quaternion &rotation, Frame *const frame) override
    {
        Q_UNUSED(rotation);
        Q_UNUSED(frame);
    };

private:
    Vec m_transConstraintDir;
    Vec m_rotConstraintDir;
    Type m_transConstraintType;
    Type m_rotConstraintType;
};

class LocalConstraint : public AxisPlaneConstraint
{
public:
    virtual ~LocalConstraint() {}

    void constrainTranslation(Vec &translation, Frame *const frame) override;
    void constrainRotation(Quaternion &rotation, Frame *const frame) override;
};

class WorldConstraint : public AxisPlaneConstraint
{
public:
    virtual ~WorldConstraint() {}

    void constrainTranslation(Vec &translation, Frame *const frame) override;
    void constrainRotation(Quaternion &rotation, Frame *const frame) override;
};

class CameraConstraint : public AxisPlaneConstraint
{
public:
    explicit CameraConstraint(const Camera *const camera);
    virtual ~CameraConstraint() {}

    void constrainTranslation(Vec &translation, Frame *const frame) override;
    void constrainRotation(Quaternion &rotation, Frame *const frame) override;

    const Camera *camera() const { return camera_; };

private:
    const Camera *const camera_;
};

} // namespace viewer
