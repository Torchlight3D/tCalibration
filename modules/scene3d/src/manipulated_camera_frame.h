#pragma once

#include "manipulated_frame.h"

namespace viewer {

class ManipulatedCameraFrame : public ManipulatedFrame
{
    friend class Camera;
    friend class ::QtOpenGLViewer;

    Q_OBJECT

public:
    ManipulatedCameraFrame();
    ManipulatedCameraFrame(const ManipulatedCameraFrame &other);
    ManipulatedCameraFrame &operator=(const ManipulatedCameraFrame &other);
    virtual ~ManipulatedCameraFrame() {}

    void setPivotPoint(const Vec &point);
    Vec pivotPoint() const;

    bool rotatesAroundUpVector() const;
    void setRotatesAroundUpVector(bool constrained);
    bool zoomsOnPivotPoint() const;

    void setZoomsOnPivotPoint(bool enabled);

public:
    qreal flySpeed() const;
    Q_SLOT void setFlySpeed(qreal speed);

    Vec sceneUpVector() const;
    Q_SLOT void setSceneUpVector(const Vec &up);

    QDomElement domElement(const QString &name,
                           QDomDocument &document) const override;
    Q_SLOT void initFromDOMElement(const QDomElement &element) override;

protected:
    void mouseReleaseEvent(QMouseEvent *const event,
                           Camera *const camera) override;
    void mouseMoveEvent(QMouseEvent *const event,
                        Camera *const camera) override;
    void wheelEvent(QWheelEvent *const event, Camera *const camera) override;

    // Why not use MouseAction enum
    void startAction(int mouseAction, bool withConstraint = true) override;

protected slots:
    void spin() override;

private slots:
    virtual void flyUpdate();

private:
    void zoom(qreal delta, const Camera *const camera);
    void updateSceneUpVector();
    Quaternion turnQuaternion(int x, const Camera *const camera);
    Quaternion pitchYawQuaternion(int x, int y, const Camera *const camera);

private:
    // Fly mode data
    qreal flySpeed_;
    qreal driveSpeed_;
    Vec sceneUpVector_;
    QTimer flyTimer_;

    bool rotatesAroundUpVector_;
    // Inverse the direction of an horizontal mouse motion. Depends on the
    // projected screen orientation of the vertical axis when the mouse button
    // is pressed.
    bool constrainedRotationIsReversed_;

    bool zoomsOnPivotPoint_;

    Vec pivotPoint_;
};

} // namespace viewer
