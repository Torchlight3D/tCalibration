#include "constraint.h"

#include "camera.h"
#include "frame.h"
#include "manipulated_camera_frame.h"

using namespace viewer;

AxisPlaneConstraint::AxisPlaneConstraint()
    : m_transConstraintType(Type::Free), m_rotConstraintType(Type::Free)
{
    // Do not use set since setRotationConstraintType needs a read.
}

void AxisPlaneConstraint::setTranslationConstraint(Type type,
                                                   const Vec &direction)
{
    setTranslationConstraintType(type);
    setTranslationConstraintDirection(direction);
}

void AxisPlaneConstraint::setTranslationConstraintDirection(
    const Vec &direction)
{
    if ((translationConstraintType() != Type::Free) &&
        (translationConstraintType() != Type::Forbidden)) {
        const qreal norm = direction.norm();
        if (norm < 1e-8) {
            qWarning() << "AxisPlaneConstraint::setTranslationConstraintDir: "
                          "null vector for translation constraint";
            m_transConstraintType = Type::Free;
        }
        else {
            m_transConstraintDir = direction / norm;
        }
    }
}

void AxisPlaneConstraint::setRotationConstraint(Type type, const Vec &direction)
{
    setRotationConstraintType(type);
    setRotationConstraintDirection(direction);
}

void AxisPlaneConstraint::setRotationConstraintDirection(const Vec &direction)
{
    if ((rotationConstraintType() != Type::Free) &&
        (rotationConstraintType() != Type::Forbidden)) {
        const qreal norm = direction.norm();
        if (norm < 1e-8) {
            qWarning() << "Null vector for rotation constraint";
            m_rotConstraintType = Type::Free;
        }
        else {
            m_rotConstraintDir = direction / norm;
        }
    }
}

void AxisPlaneConstraint::setRotationConstraintType(Type type)
{
    if (rotationConstraintType() == Type::Plane) {
        qWarning()
            << "The PLANE type cannot be used for a rotation constraints";
        return;
    }

    m_rotConstraintType = type;
}

void LocalConstraint::constrainTranslation(Vec &translation, Frame *const frame)
{
    switch (translationConstraintType()) {
        case Type::Free:
            break;
        case Type::Plane: {
            const auto proj =
                frame->rotation().rotate(translationConstraintDirection());
            translation.projectOnPlane(proj);
        } break;
        case Type::Axis: {
            const auto proj =
                frame->rotation().rotate(translationConstraintDirection());
            translation.projectOnAxis(proj);
        } break;
        case Type::Forbidden:
            translation = {};
            break;
        default:
            break;
    }
}

void LocalConstraint::constrainRotation(Quaternion &rotation, Frame *const)
{
    switch (rotationConstraintType()) {
        case Type::Free:
            break;
        case Type::Plane:
            break;
        case Type::Axis: {
            Vec axis = rotationConstraintDirection();
            Vec quat{rotation[0], rotation[1], rotation[2]};
            quat.projectOnAxis(axis);
            rotation = {quat, 2.0 * acos(rotation[3])};
        } break;
        case Type::Forbidden:
            rotation = {};
            break;
        default:
            break;
    }
}

void WorldConstraint::constrainTranslation(Vec &translation, Frame *const frame)
{
    switch (translationConstraintType()) {
        case Type::Free:
            break;
        case Type::Plane:
            if (frame->referenceFrame()) {
                const auto proj = frame->referenceFrame()->transformOf(
                    translationConstraintDirection());
                translation.projectOnPlane(proj);
            }
            else
                translation.projectOnPlane(translationConstraintDirection());
            break;
        case Type::Axis:
            if (frame->referenceFrame()) {
                const auto proj = frame->referenceFrame()->transformOf(
                    translationConstraintDirection());
                translation.projectOnAxis(proj);
            }
            else
                translation.projectOnAxis(translationConstraintDirection());
            break;
        case Type::Forbidden:
            translation = {};
            break;
        default:
            break;
    }
}

void WorldConstraint::constrainRotation(Quaternion &rotation,
                                        Frame *const frame)
{
    switch (rotationConstraintType()) {
        case Type::Free:
            break;
        case Type::Plane:
            break;
        case Type::Axis: {
            Vec quat(rotation[0], rotation[1], rotation[2]);
            Vec axis = frame->transformOf(rotationConstraintDirection());
            quat.projectOnAxis(axis);
            rotation = Quaternion(quat, 2.0 * acos(rotation[3]));
            break;
        }
        case Type::Forbidden:
            rotation = {};
            break;
        default:
            break;
    }
}

CameraConstraint::CameraConstraint(const Camera *const camera)
    : AxisPlaneConstraint(), camera_(camera)
{
}

void CameraConstraint::constrainTranslation(Vec &translation,
                                            Frame *const frame)
{
    switch (translationConstraintType()) {
        case Type::Free:
            break;
        case Type::Plane: {
            auto proj = camera()->frame()->inverseTransformOf(
                translationConstraintDirection());
            if (frame->referenceFrame())
                proj = frame->referenceFrame()->transformOf(proj);
            translation.projectOnPlane(proj);
        } break;
        case Type::Axis: {
            auto proj = camera()->frame()->inverseTransformOf(
                translationConstraintDirection());
            if (frame->referenceFrame())
                proj = frame->referenceFrame()->transformOf(proj);
            translation.projectOnAxis(proj);
        } break;
        case Type::Forbidden:
            translation = {};
            break;
        default:
            break;
    }
}

void CameraConstraint::constrainRotation(Quaternion &rotation,
                                         Frame *const frame)
{
    switch (rotationConstraintType()) {
        case Type::Free:
            break;
        case Type::Plane:
            break;
        case Type::Axis: {
            Vec axis = frame->transformOf(camera()->frame()->inverseTransformOf(
                rotationConstraintDirection()));
            Vec quat = Vec(rotation[0], rotation[1], rotation[2]);
            quat.projectOnAxis(axis);
            rotation = Quaternion(quat, 2.0 * acos(rotation[3]));
        } break;
        case Type::Forbidden:
            rotation = {};
            break;
    }
}
