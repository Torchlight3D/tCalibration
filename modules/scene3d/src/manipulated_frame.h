#pragma once

#include <QDateTime>
#include <QElapsedTimer>
#include <QString>

#include "frame.h"
#include "mouse_grabber.h"
#include "viewer.h"

namespace viewer {

class ManipulatedFrame : public Frame, public MouseGrabber
{
    friend class Camera;
    friend class ::QtOpenGLViewer;

    Q_OBJECT

public:
    // FIXME: Potential memory leaks
    ManipulatedFrame();
    ManipulatedFrame(const ManipulatedFrame &other);
    ManipulatedFrame &operator=(const ManipulatedFrame &other);
    virtual ~ManipulatedFrame() {}

    qreal rotationSensitivity() const;
    Q_SLOT void setRotationSensitivity(qreal sensitivity);

    qreal translationSensitivity() const;
    Q_SLOT void setTranslationSensitivity(qreal sensitivity);

    qreal spinningSensitivity() const;
    Q_SLOT void setSpinningSensitivity(qreal sensitivity);

    qreal zoomSensitivity() const;
    Q_SLOT void setZoomSensitivity(qreal sensitivity);

    qreal wheelSensitivity() const;
    Q_SLOT void setWheelSensitivity(qreal sensitivity);

    Quaternion spinningQuaternion() const;
    Q_SLOT void setSpinningQuaternion(const Quaternion &quat);

    bool isSpinning() const;
    Q_SLOT virtual void startSpinning(int updateInterval);
    Q_SLOT virtual void stopSpinning();

signals:
    void manipulated();
    void spun();

protected slots:
    virtual void spin();

private slots:
    void spinUpdate();

protected:
    void mousePressEvent(QMouseEvent *const event,
                         Camera *const camera) override;
    void mouseMoveEvent(QMouseEvent *const event,
                        Camera *const camera) override;
    void mouseReleaseEvent(QMouseEvent *const event,
                           Camera *const camera) override;
    void mouseDoubleClickEvent(QMouseEvent *const event,
                               Camera *const camera) override;
    void wheelEvent(QWheelEvent *const event, Camera *const camera) override;

public:
    bool isManipulated() const;
    
    QtOpenGLViewer::MouseAction currentMouseAction() const { return action_; }

public:
    void checkIfGrabsMouse(int x, int y, const Camera *const camera) override;

public:
    QDomElement domElement(const QString &name,
                           QDomDocument &document) const override;
    Q_SLOT void initFromDOMElement(const QDomElement &element) override;

protected:
    Quaternion deformedBallQuaternion(int x, int y, qreal cx, qreal cy,
                                      const Camera *const camera);

    // int is actually a QGLViewer::MouseAction
    virtual void startAction(int ma, bool withConstraint = true);
    void computeMouseSpeed(const QMouseEvent *const e);
    int mouseOriginalDirection(const QMouseEvent *const e);

    qreal deltaWithPrevPos(QMouseEvent *const event,
                           Camera *const camera) const;

    qreal wheelDelta(const QWheelEvent *event) const;

protected:
    QtOpenGLViewer::MouseAction action_;
    Constraint *previousConstraint_; // When manipulation is without Contraint.
    QPoint prevPos_, pressPos_;

private:
    void zoom(qreal delta, const Camera *const camera);

private:
    // Sensitivity
    qreal rotationSensitivity_;
    qreal translationSensitivity_;
    qreal spinningSensitivity_;
    qreal wheelSensitivity_;
    qreal zoomSensitivity_;

    // Mouse speed and spinning
    QElapsedTimer last_move_time;
    qreal mouseSpeed_;
    int delay_;
    bool isSpinning_;
    QTimer spinningTimer_;
    Quaternion spinningQuaternion_;

    // Whether the SCREEN_TRANS direction (horizontal or vertical) is fixed or
    // not.
    bool dirIsFixed_;

    // MouseGrabber
    bool keepsGrabbingMouse_;
};

} // namespace viewer
