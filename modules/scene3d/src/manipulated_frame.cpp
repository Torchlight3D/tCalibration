#include "manipulated_frame.h"

#include <QMouseEvent>

#include "manipulated_camera_frame.h"
#include "camera.h"
#include "dom_utils.h"
#include "viewer.h"

using namespace viewer;

ManipulatedFrame::ManipulatedFrame()
    : action_(QtOpenGLViewer::NoMouseAction), keepsGrabbingMouse_(false)
{
    setRotationSensitivity(1.0);
    setTranslationSensitivity(1.0);
    setSpinningSensitivity(0.3);
    setWheelSensitivity(1.0);
    setZoomSensitivity(1.0);

    isSpinning_ = false;
    previousConstraint_ = nullptr;

    connect(&spinningTimer_, &QTimer::timeout, this,
            &ManipulatedFrame::spinUpdate);
}

ManipulatedFrame &ManipulatedFrame::operator=(const ManipulatedFrame &mf)
{
    Frame::operator=(mf);

    setRotationSensitivity(mf.rotationSensitivity());
    setTranslationSensitivity(mf.translationSensitivity());
    setSpinningSensitivity(mf.spinningSensitivity());
    setWheelSensitivity(mf.wheelSensitivity());
    setZoomSensitivity(mf.zoomSensitivity());

    mouseSpeed_ = 0.0;
    dirIsFixed_ = false;
    keepsGrabbingMouse_ = false;
    action_ = QtOpenGLViewer::NoMouseAction;

    return *this;
}

ManipulatedFrame::ManipulatedFrame(const ManipulatedFrame &mf)
    : Frame(mf), MouseGrabber()
{
    (*this) = mf;
}

void ManipulatedFrame::checkIfGrabsMouse(int x, int y,
                                         const Camera *const camera)
{
    const int thresold = 10;
    const Vec proj = camera->projectedCoordinatesOf(position());
    setGrabsMouse(keepsGrabbingMouse_ || ((std::abs(x - proj.x) < thresold) &&
                                          (std::abs(y - proj.y) < thresold)));
}

namespace key {
constexpr char kTagName[]{"ManipulatedParameters"};
constexpr char kRotationSensitivity[]{"rotSens"};
constexpr char kTranslationSensitivity[]{"transSens"};
constexpr char kSpinSensitivity[]{"spinSens"};
constexpr char kWheelSensitivity[]{"wheelSens"};
constexpr char kZoomSensitivity[]{"zoomSens"};
} // namespace key

QDomElement ManipulatedFrame::domElement(const QString &name,
                                         QDomDocument &doc) const
{
    auto mp = doc.createElement(key::kTagName);
    mp.setAttribute(key::kRotationSensitivity,
                    QString::number(rotationSensitivity()));
    mp.setAttribute(key::kTranslationSensitivity,
                    QString::number(translationSensitivity()));
    mp.setAttribute(key::kSpinSensitivity,
                    QString::number(spinningSensitivity()));
    mp.setAttribute(key::kWheelSensitivity,
                    QString::number(wheelSensitivity()));
    mp.setAttribute(key::kZoomSensitivity, QString::number(zoomSensitivity()));

    auto elem = Frame::domElement(name, doc);
    elem.appendChild(mp);

    return elem;
}

void ManipulatedFrame::initFromDOMElement(const QDomElement &element)
{
    // Not called since it would set constraint() and referenceFrame() to
    // nullptr. *this = ManipulatedFrame();
    Frame::initFromDOMElement(element);

    stopSpinning();

    auto child = element.firstChild().toElement();
    while (!child.isNull()) {
        if (child.tagName() == key::kTagName) {
            setRotationSensitivity(
                io::qrealFromDom(child, key::kRotationSensitivity, 1.0));
            setTranslationSensitivity(
                io::qrealFromDom(child, key::kTranslationSensitivity, 1.0));
            setSpinningSensitivity(
                io::qrealFromDom(child, key::kSpinSensitivity, 0.3));
            setWheelSensitivity(
                io::qrealFromDom(child, key::kWheelSensitivity, 1.0));
            setZoomSensitivity(
                io::qrealFromDom(child, key::kZoomSensitivity, 1.0));
        }
        child = child.nextSibling().toElement();
    }
}

bool ManipulatedFrame::isManipulated() const
{
    return action_ != QtOpenGLViewer::NoMouseAction;
}

void ManipulatedFrame::startSpinning(int updateInterval)
{
    isSpinning_ = true;
    spinningTimer_.start(updateInterval);
}

void ManipulatedFrame::stopSpinning()
{
    spinningTimer_.stop();
    isSpinning_ = false;
}

// spin() and spinUpdate() differ since spin can be used by itself (for instance
// by QGLViewer::SCREEN_ROTATE) without a spun emission. Much nicer to use the
// spinningQuaternion() and hence spin() for these incremental updates. Nothing
// special to be done for continuous spinning with this design.

void ManipulatedFrame::spin() { rotate(spinningQuaternion()); }

void ManipulatedFrame::spinUpdate()
{
    spin();
    emit spun();
}

void ManipulatedFrame::startAction(int ma, bool withConstraint)
{
    action_ = static_cast<QtOpenGLViewer::MouseAction>(ma);

    // #CONNECTION# manipulatedFrame::wheelEvent,
    // manipulatedCameraFrame::wheelEvent and mouseReleaseEvent() restore
    // previous constraint
    if (withConstraint)
        previousConstraint_ = nullptr;
    else {
        previousConstraint_ = constraint();
        setConstraint(nullptr);
    }

    switch (action_) {
        case QtOpenGLViewer::Rotate:
        case QtOpenGLViewer::ScreenRotate:
            mouseSpeed_ = 0.0;
            stopSpinning();
            break;

        case QtOpenGLViewer::ScreenTranslate:
            dirIsFixed_ = false;
            break;

        default:
            break;
    }
}

void ManipulatedFrame::computeMouseSpeed(const QMouseEvent *const e)
{
    const QPoint delta = (e->pos() - prevPos_);
    const qreal dist =
        sqrt(qreal(delta.x() * delta.x() + delta.y() * delta.y()));
    delay_ = last_move_time.restart();
    if (delay_ == 0)
        // Less than a millisecond: assume delay = 1ms
        mouseSpeed_ = dist;
    else
        mouseSpeed_ = dist / delay_;
}

int ManipulatedFrame::mouseOriginalDirection(const QMouseEvent *const e)
{
    // Two simultaneous manipulatedFrame require two mice !
    static bool horiz = true;

    if (!dirIsFixed_) {
        const QPoint delta = e->pos() - pressPos_;
        dirIsFixed_ = std::abs(delta.x()) != std::abs(delta.y());
        horiz = std::abs(delta.x()) > std::abs(delta.y());
    }

    return dirIsFixed_ ? (horiz ? 1 : -1) : 0;
}

qreal ManipulatedFrame::deltaWithPrevPos(QMouseEvent *const event,
                                         Camera *const camera) const
{
    qreal dx =
        qreal(event->position().x() - prevPos_.x()) / camera->screenWidth();
    qreal dy =
        qreal(event->position().y() - prevPos_.y()) / camera->screenHeight();

    qreal value = std::abs(dx) > std::abs(dy) ? dx : dy;
    return value * zoomSensitivity();
}

qreal ManipulatedFrame::wheelDelta(const QWheelEvent *event) const
{
    constexpr qreal kWheelSensitivityCoeff{8e-4};
    return event->angleDelta().y() * wheelSensitivity() *
           kWheelSensitivityCoeff;
}

void ManipulatedFrame::zoom(qreal delta, const Camera *const camera)
{
    Vec trans(0.0, 0.0, (camera->position() - position()).norm() * delta);

    trans = camera->frame()->orientation().rotate(trans);
    if (referenceFrame())
        trans = referenceFrame()->transformOf(trans);
    translate(trans);
}

void ManipulatedFrame::mousePressEvent(QMouseEvent *const event,
                                       Camera *const camera)
{
    Q_UNUSED(camera)

    if (grabsMouse())
        keepsGrabbingMouse_ = true;

    // #CONNECTION setMouseBinding
    // action_ should no longer possibly be NO_MOUSE_ACTION since this value is
    // not inserted in mouseBinding_
    // if (action_ == QGLViewer::NO_MOUSE_ACTION)
    // event->ignore();

    prevPos_ = pressPos_ = event->pos();
}

void ManipulatedFrame::mouseMoveEvent(QMouseEvent *const event,
                                      Camera *const camera)
{
    switch (action_) {
        case QtOpenGLViewer::Translate: {
            const QPoint delta = event->pos() - prevPos_;
            Vec trans(delta.x(), -delta.y(), 0.0);
            // Scale to fit the screen mouse displacement
            switch (camera->type()) {
                case Camera::Type::Perspective:
                    trans *=
                        2.0 * tan(camera->fieldOfView() / 2.0) *
                        std::abs(camera->frame()->coordinatesOf(position()).z) /
                        camera->screenHeight();
                    break;
                case Camera::Type::Orthographic: {
                    GLdouble w, h;
                    camera->getOrthoWidthHeight(w, h);
                    trans[0] *= 2.0 * w / camera->screenWidth();
                    trans[1] *= 2.0 * h / camera->screenHeight();
                    break;
                }
            }
            // Transform to world coordinate system.
            trans = camera->frame()->orientation().rotate(
                translationSensitivity() * trans);
            // And then down to frame
            if (referenceFrame())
                trans = referenceFrame()->transformOf(trans);
            translate(trans);
            break;
        }
        case QtOpenGLViewer::Zoom: {
            zoom(deltaWithPrevPos(event, camera), camera);
            break;
        }
        case QtOpenGLViewer::ScreenRotate: {
            Vec trans = camera->projectedCoordinatesOf(position());

            const qreal prev_angle =
                atan2(prevPos_.y() - trans[1], prevPos_.x() - trans[0]);

            const qreal angle = atan2(event->position().y() - trans[1],
                                      event->position().x() - trans[0]);

            const Vec axis = transformOf(
                camera->frame()->inverseTransformOf(Vec(0.0, 0.0, -1.0)));
            Quaternion rot(axis, angle - prev_angle);
            // #CONNECTION# These two methods should go together (spinning
            // detection and
            //  activation)
            computeMouseSpeed(event);
            setSpinningQuaternion(rot);
            spin();
            break;
        }

        case QtOpenGLViewer::ScreenTranslate: {
            Vec trans;
            int dir = mouseOriginalDirection(event);
            if (dir == 1)
                trans.setValue(event->position().x() - prevPos_.x(), 0.0, 0.0);
            else if (dir == -1)
                trans.setValue(0.0, prevPos_.y() - event->position().y(), 0.0);

            switch (camera->type()) {
                case Camera::Type::Perspective:
                    trans *=
                        2.0 * std::tan(camera->fieldOfView() / 2.0) *
                        std::abs(camera->frame()->coordinatesOf(position()).z) /
                        camera->screenHeight();
                    break;
                case Camera::Type::Orthographic: {
                    GLdouble w, h;
                    camera->getOrthoWidthHeight(w, h);
                    trans[0] *= 2.0 * w / camera->screenWidth();
                    trans[1] *= 2.0 * h / camera->screenHeight();
                    break;
                }
            }
            // Transform to world coordinate system.
            trans = camera->frame()->orientation().rotate(
                translationSensitivity() * trans);
            // And then down to frame
            if (referenceFrame())
                trans = referenceFrame()->transformOf(trans);

            translate(trans);
            break;
        }

        case QtOpenGLViewer::Rotate: {
            Vec trans = camera->projectedCoordinatesOf(position());
            Quaternion rot = deformedBallQuaternion(event->position().x(),
                                                    event->position().y(),
                                                    trans[0], trans[1], camera);

            trans = Vec(-rot[0], -rot[1], -rot[2]);
            trans = camera->frame()->orientation().rotate(trans);
            trans = transformOf(trans);
            rot[0] = trans[0];
            rot[1] = trans[1];
            rot[2] = trans[2];
            // #CONNECTION# These two methods should go together (spinning
            // detection and
            //  activation)
            computeMouseSpeed(event);
            setSpinningQuaternion(rot);
            spin();
            break;
        }

        case QtOpenGLViewer::MoveForward:
        case QtOpenGLViewer::MoveBackward:
        case QtOpenGLViewer::LookForward:
        case QtOpenGLViewer::Roll:
        case QtOpenGLViewer::Drive:
        case QtOpenGLViewer::ZoomOnRegion:
            // These MouseAction values make no sense for a manipulatedFrame
            break;
        case QtOpenGLViewer::NoMouseAction:
            // Possible when the ManipulatedFrame is a MouseGrabber. This method
            // is then called without startAction because of mouseTracking.
            break;
    }

    if (action_ != QtOpenGLViewer::NoMouseAction) {
        prevPos_ = event->pos();
        emit manipulated();
    }
}

void ManipulatedFrame::mouseReleaseEvent(QMouseEvent *const event,
                                         Camera *const camera)
{
    Q_UNUSED(event)
    Q_UNUSED(camera)

    keepsGrabbingMouse_ = false;

    if (previousConstraint_)
        setConstraint(previousConstraint_);

    if (((action_ == QtOpenGLViewer::Rotate) ||
         (action_ == QtOpenGLViewer::ScreenRotate)) &&
        (mouseSpeed_ >= spinningSensitivity()))
        startSpinning(delay_);

    action_ = QtOpenGLViewer::NoMouseAction;
}

void ManipulatedFrame::mouseDoubleClickEvent(QMouseEvent *const event,
                                             Camera *const camera)
{
    if (event->modifiers() == Qt::NoModifier)
        switch (event->button()) {
            case Qt::LeftButton:
                alignWithFrame(camera->frame());
                break;
            case Qt::RightButton:
                projectOnLine(camera->position(), camera->viewDirection());
                break;
            default:
                break;
        }
}

void ManipulatedFrame::wheelEvent(QWheelEvent *const event,
                                  Camera *const camera)
{
    // #CONNECTION# QGLViewer::setWheelBinding
    if (action_ == QtOpenGLViewer::Zoom) {
        zoom(wheelDelta(event), camera);
        emit manipulated();
    }

    // #CONNECTION# startAction should always be called before
    if (previousConstraint_)
        setConstraint(previousConstraint_);

    action_ = QtOpenGLViewer::NoMouseAction;
}

static qreal projectOnBall(qreal x, qreal y)
{
    // If you change the size value, change angle computation in
    // deformedBallQuaternion().
    const qreal size = 1.0;
    const qreal size2 = size * size;
    const qreal size_limit = size2 * 0.5;

    const qreal d = x * x + y * y;
    return d < size_limit ? sqrt(size2 - d) : size_limit / sqrt(d);
}

Quaternion ManipulatedFrame::deformedBallQuaternion(int x, int y, qreal cx,
                                                    qreal cy,
                                                    const Camera *const camera)
{
    // Points on the deformed ball
    qreal px =
        rotationSensitivity() * (prevPos_.x() - cx) / camera->screenWidth();
    qreal py =
        rotationSensitivity() * (cy - prevPos_.y()) / camera->screenHeight();
    qreal dx = rotationSensitivity() * (x - cx) / camera->screenWidth();
    qreal dy = rotationSensitivity() * (cy - y) / camera->screenHeight();

    const Vec p1(px, py, projectOnBall(px, py));
    const Vec p2(dx, dy, projectOnBall(dx, dy));
    // Approximation of rotation angle
    // Should be divided by the projectOnBall size, but it is 1.0
    const Vec axis = cross(p2, p1);
    const qreal angle =
        5.0 *
        asin(sqrt(axis.squaredNorm() / p1.squaredNorm() / p2.squaredNorm()));
    return Quaternion(axis, angle);
}

void ManipulatedFrame::setRotationSensitivity(qreal sensitivity)
{
    rotationSensitivity_ = sensitivity;
}

void ManipulatedFrame::setTranslationSensitivity(qreal sensitivity)
{
    translationSensitivity_ = sensitivity;
}

void ManipulatedFrame::setSpinningSensitivity(qreal sensitivity)
{
    spinningSensitivity_ = sensitivity;
}

void ManipulatedFrame::setWheelSensitivity(qreal sensitivity)
{
    wheelSensitivity_ = sensitivity;
}

void ManipulatedFrame::setZoomSensitivity(qreal sensitivity)
{
    zoomSensitivity_ = sensitivity;
}

qreal ManipulatedFrame::rotationSensitivity() const
{
    return rotationSensitivity_;
}
qreal ManipulatedFrame::translationSensitivity() const
{
    return translationSensitivity_;
}
qreal ManipulatedFrame::spinningSensitivity() const
{
    return spinningSensitivity_;
}
qreal ManipulatedFrame::zoomSensitivity() const { return zoomSensitivity_; }
qreal ManipulatedFrame::wheelSensitivity() const { return wheelSensitivity_; }

bool ManipulatedFrame::isSpinning() const { return isSpinning_; }

Quaternion ManipulatedFrame::spinningQuaternion() const
{
    return spinningQuaternion_;
}

void ManipulatedFrame::setSpinningQuaternion(const Quaternion &q)
{
    spinningQuaternion_ = q;
}
