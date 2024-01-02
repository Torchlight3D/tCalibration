#pragma once

#include <QMouseEvent>

class QtOpenGLViewer;

namespace viewer {

class Camera;

class MouseGrabber
{
public:
    MouseGrabber();
    virtual ~MouseGrabber();

    virtual void checkIfGrabsMouse(int x, int y,
                                   const Camera *const camera) = 0;

    bool grabsMouse() const;

    static const QList<MouseGrabber *> &MouseGrabberPool();

    bool isInMouseGrabberPool() const;

    void addInMouseGrabberPool();
    void removeFromMouseGrabberPool();
    void clearMouseGrabberPool(bool autoDelete = false);

protected:
    void setGrabsMouse(bool grabs);

    virtual void mousePressEvent(QMouseEvent *const event,
                                 Camera *const camera);
    virtual void mouseMoveEvent(QMouseEvent *const event, Camera *const camera);
    virtual void mouseReleaseEvent(QMouseEvent *const event,
                                   Camera *const camera);
    virtual void mouseDoubleClickEvent(QMouseEvent *const event,
                                       Camera *const camera);
    virtual void wheelEvent(QWheelEvent *const event, Camera *const camera);

private:
    Q_DISABLE_COPY(MouseGrabber)
    static QList<MouseGrabber *> MouseGrabberPool_;
    bool grabsMouse_;
    
    friend class ::QtOpenGLViewer;
};

} // namespace viewer
