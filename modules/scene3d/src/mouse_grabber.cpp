#include "mouse_grabber.h"

namespace viewer {

namespace {

}

QList<MouseGrabber *> MouseGrabber::MouseGrabberPool_;

MouseGrabber::MouseGrabber() : grabsMouse_(false) { addInMouseGrabberPool(); }

MouseGrabber::~MouseGrabber()
{
    MouseGrabber::MouseGrabberPool_.removeAll(this);
}

bool MouseGrabber::grabsMouse() const { return grabsMouse_; }

const QList<MouseGrabber *> &MouseGrabber::MouseGrabberPool()
{
    return MouseGrabber::MouseGrabberPool_;
}

bool MouseGrabber::isInMouseGrabberPool() const
{
    return MouseGrabber::MouseGrabberPool_.contains(
        const_cast<MouseGrabber *>(this));
}

void MouseGrabber::setGrabsMouse(bool grabs) { grabsMouse_ = grabs; }

void MouseGrabber::addInMouseGrabberPool()
{
    if (!isInMouseGrabberPool()) {
        MouseGrabber::MouseGrabberPool_.append(this);
    }
}

void MouseGrabber::removeFromMouseGrabberPool()
{
    if (isInMouseGrabberPool()) {
        MouseGrabber::MouseGrabberPool_.removeAll(
            const_cast<MouseGrabber *>(this));
    }
}

void MouseGrabber::clearMouseGrabberPool(bool autoDelete)
{
    if (autoDelete) {
        qDeleteAll(MouseGrabber::MouseGrabberPool_);
    }
    MouseGrabber::MouseGrabberPool_.clear();
}

void MouseGrabber::mousePressEvent(QMouseEvent *const event,
                                   Camera *const camera)
{
    Q_UNUSED(event);
    Q_UNUSED(camera);
}

void MouseGrabber::mouseDoubleClickEvent(QMouseEvent *const event,
                                         Camera *const camera)
{
    Q_UNUSED(event);
    Q_UNUSED(camera);
}

void MouseGrabber::mouseReleaseEvent(QMouseEvent *const event,
                                     Camera *const camera)
{
    Q_UNUSED(event);
    Q_UNUSED(camera);
}

void MouseGrabber::mouseMoveEvent(QMouseEvent *const event,
                                  Camera *const camera)
{
    Q_UNUSED(event);
    Q_UNUSED(camera);
}

void MouseGrabber::wheelEvent(QWheelEvent *const event, Camera *const camera)
{
    Q_UNUSED(event);
    Q_UNUSED(camera);
}

} // namespace viewer
