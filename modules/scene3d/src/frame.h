#pragma once

#include <QObject>
#include <QString>

#include "constraint.h"

namespace viewer {

class Frame : public QObject
{
    Q_OBJECT

public:
    // FIXME: Potential memory leaks
    Frame();
    Frame(const Vec &position, const Quaternion &orientation);
    Frame(const Frame &frame);
    Frame &operator=(const Frame &frame);
    virtual ~Frame() {}

    void setPosition(const Vec &position);
    inline void setPosition(qreal x, qreal y, qreal z)
    {
        setPosition({x, y, z});
    }
    void setPositionWithConstraint(const Vec &position);

    void setOrientation(const Quaternion &orientation);
    void setOrientation(qreal q0, qreal q1, qreal q2, qreal q3);
    void setOrientationWithConstraint(const Quaternion &orientation);

    void setPositionAndOrientation(const Vec &position,
                                   const Quaternion &orientation);
    void setPositionAndOrientationWithConstraint(const Vec &position,
                                                 const Quaternion &orientation);

    Vec position() const;
    Quaternion orientation() const;

    void getPosition(qreal &x, qreal &y, qreal &z) const;
    void getOrientation(qreal &q0, qreal &q1, qreal &q2, qreal &q3) const;

    void setTranslation(const Vec &translation);
    inline void setTranslation(qreal x, qreal y, qreal z)
    {
        setTranslation({x, y, z});
    }
    void setTranslationWithConstraint(const Vec &translation);

    void setRotation(const Quaternion &rotation);
    inline void setRotation(qreal q0, qreal q1, qreal q2, qreal q3)
    {
        setRotation({q0, q1, q2, q3});
    }
    void setRotationWithConstraint(const Quaternion &rotation);

    void setTranslationAndRotation(const Vec &translation,
                                   const Quaternion &rotation);
    void setTranslationAndRotationWithConstraint(const Vec &translation,
                                                 const Quaternion &rotation);

    Vec translation() const;

    Quaternion rotation() const;

    void getTranslation(qreal &x, qreal &y, qreal &z) const;
    void getRotation(qreal &q0, qreal &q1, qreal &q2, qreal &q3) const;

    const Frame *referenceFrame() const { return referenceFrame_; }
    void setReferenceFrame(const Frame *const refFrame);
    bool settingAsReferenceFrameWillCreateALoop(const Frame *const frame);

    void translate(Vec &t);
    void translate(const Vec &t);
    // Some compilers complain about "overloading cannot distinguish from
    // previous declaration" Simply comment out the following method and its
    // associated implementation
    void translate(qreal x, qreal y, qreal z);
    void translate(qreal &x, qreal &y, qreal &z);

    void rotate(Quaternion &q);
    void rotate(const Quaternion &q);
    // Some compilers complain about "overloading cannot distinguish from
    // previous declaration" Simply comment out the following method and its
    // associated implementation
    void rotate(qreal q0, qreal q1, qreal q2, qreal q3);
    void rotate(qreal &q0, qreal &q1, qreal &q2, qreal &q3);

    void rotateAroundPoint(Quaternion &rotation, const Vec &point);
    void rotateAroundPoint(const Quaternion &rotation, const Vec &point);

    void alignWithFrame(const Frame *const frame, bool move = false,
                        qreal threshold = 0.0);
    void projectOnLine(const Vec &origin, const Vec &direction);

    Vec coordinatesOf(const Vec &src) const;
    Vec inverseCoordinatesOf(const Vec &src) const;
    Vec localCoordinatesOf(const Vec &src) const;
    Vec localInverseCoordinatesOf(const Vec &src) const;
    Vec coordinatesOfIn(const Vec &src, const Frame *const in) const;
    Vec coordinatesOfFrom(const Vec &src, const Frame *const from) const;

    void getCoordinatesOf(const qreal src[3], qreal res[3]) const;
    void getInverseCoordinatesOf(const qreal src[3], qreal res[3]) const;
    void getLocalCoordinatesOf(const qreal src[3], qreal res[3]) const;
    void getLocalInverseCoordinatesOf(const qreal src[3], qreal res[3]) const;
    void getCoordinatesOfIn(const qreal src[3], qreal res[3],
                            const Frame *const in) const;
    void getCoordinatesOfFrom(const qreal src[3], qreal res[3],
                              const Frame *const from) const;

    Vec transformOf(const Vec &src) const;
    Vec inverseTransformOf(const Vec &src) const;
    Vec localTransformOf(const Vec &src) const;
    Vec localInverseTransformOf(const Vec &src) const;
    Vec transformOfIn(const Vec &src, const Frame *const in) const;
    Vec transformOfFrom(const Vec &src, const Frame *const from) const;

    void getTransformOf(const qreal src[3], qreal res[3]) const;
    void getInverseTransformOf(const qreal src[3], qreal res[3]) const;
    void getLocalTransformOf(const qreal src[3], qreal res[3]) const;
    void getLocalInverseTransformOf(const qreal src[3], qreal res[3]) const;
    void getTransformOfIn(const qreal src[3], qreal res[3],
                          const Frame *const in) const;
    void getTransformOfFrom(const qreal src[3], qreal res[3],
                            const Frame *const from) const;

    Constraint *constraint() const;

    void setConstraint(Constraint *const constraint);

    const GLdouble *matrix() const;
    void getMatrix(GLdouble m[4][4]) const;
    void getMatrix(GLdouble m[16]) const;

    const GLdouble *worldMatrix() const;
    void getWorldMatrix(GLdouble m[4][4]) const;
    void getWorldMatrix(GLdouble m[16]) const;

    void setFromMatrix(const GLdouble m[4][4]);
    void setFromMatrix(const GLdouble m[16]);

    Frame inverse() const;
    Frame worldInverse() const;

    virtual QDomElement domElement(const QString &name,
                                   QDomDocument &document) const;

signals:
    void modified();
    void interpolated();

public slots:
    virtual void initFromDOMElement(const QDomElement &element);

private:
    Vec t_;
    Quaternion q_;
    Constraint *constraint_;
    const Frame *referenceFrame_;
};

} // namespace viewer
