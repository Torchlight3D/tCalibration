#include "manipulated_camera_frame.h"

#include <QMouseEvent>

#include "camera.h"
#include "dom_utils.h"
#include "viewer.h"

using namespace viewer;

ManipulatedCameraFrame::ManipulatedCameraFrame()
    : driveSpeed_(0.0),
      sceneUpVector_(0.0, 1.0, 0.0),
      rotatesAroundUpVector_(false),
      zoomsOnPivotPoint_(false)
{
    setFlySpeed(0.0);
    removeFromMouseGrabberPool();
    connect(&flyTimer_, &QTimer::timeout, this,
            &ManipulatedCameraFrame::flyUpdate);
}

ManipulatedCameraFrame &ManipulatedCameraFrame::operator=(
    const ManipulatedCameraFrame &mcf)
{
    ManipulatedFrame::operator=(mcf);

    setFlySpeed(mcf.flySpeed());
    setSceneUpVector(mcf.sceneUpVector());
    setRotatesAroundUpVector(mcf.rotatesAroundUpVector_);
    setZoomsOnPivotPoint(mcf.zoomsOnPivotPoint_);

    return *this;
}

ManipulatedCameraFrame::ManipulatedCameraFrame(
    const ManipulatedCameraFrame &mcf)
    : ManipulatedFrame(mcf)
{
    removeFromMouseGrabberPool();
    connect(&flyTimer_, &QTimer::timeout, this,
            &ManipulatedCameraFrame::flyUpdate);
    (*this) = (mcf);
}

void ManipulatedCameraFrame::setPivotPoint(const Vec &point)
{
    pivotPoint_ = point;
}

Vec ManipulatedCameraFrame::pivotPoint() const { return pivotPoint_; }

bool ManipulatedCameraFrame::rotatesAroundUpVector() const
{
    return rotatesAroundUpVector_;
}

void ManipulatedCameraFrame::setRotatesAroundUpVector(bool constrained)
{
    rotatesAroundUpVector_ = constrained;
}

bool ManipulatedCameraFrame::zoomsOnPivotPoint() const
{
    return zoomsOnPivotPoint_;
}

void ManipulatedCameraFrame::setZoomsOnPivotPoint(bool enabled)
{
    zoomsOnPivotPoint_ = enabled;
}

void ManipulatedCameraFrame::setFlySpeed(qreal speed) { flySpeed_ = speed; }
void ManipulatedCameraFrame::setSceneUpVector(const Vec &up)
{
    sceneUpVector_ = up;
}

qreal ManipulatedCameraFrame::flySpeed() const { return flySpeed_; }
Vec ManipulatedCameraFrame::sceneUpVector() const { return sceneUpVector_; }

void ManipulatedCameraFrame::spin()
{
    rotateAroundPoint(spinningQuaternion(), pivotPoint());
}

void ManipulatedCameraFrame::flyUpdate()
{
    static Vec flyDisp{};
    switch (action_) {
        case QtOpenGLViewer::MoveForward:
            flyDisp.z = -flySpeed();
            translate(localInverseTransformOf(flyDisp));
            break;
        case QtOpenGLViewer::MoveBackward:
            flyDisp.z = flySpeed();
            translate(localInverseTransformOf(flyDisp));
            break;
        case QtOpenGLViewer::Drive:
            flyDisp.z = flySpeed() * driveSpeed_;
            translate(localInverseTransformOf(flyDisp));
            break;
        default:
            break;
    }

    // Needs to be out of the switch since ZOOM/fastDraw()/wheelEvent use this
    // callback to trigger a final draw(). #CONNECTION# wheelEvent.
    emit manipulated();
}

void ManipulatedCameraFrame::updateSceneUpVector()
{
    sceneUpVector_ = inverseTransformOf(Vec(0.0, 1.0, 0.0));
}

namespace key {
constexpr char kParameters[]{"ManipulatedCameraParameters"};
constexpr char kFlySpeed[]{"flySpeed"};
constexpr char kRotationAroundUpVector[]{"rotatesAroundUpVector"};
constexpr char kZoomOnPivot[]{"zoomsOnPivotPoint"};
constexpr char kSceneupVector[]{"sceneUpVector"};
} // namespace key

QDomElement ManipulatedCameraFrame::domElement(const QString &name,
                                               QDomDocument &doc) const
{
    auto mcp = doc.createElement(key::kParameters);
    mcp.setAttribute(key::kFlySpeed, QString::number(flySpeed()));
    io::setBoolAttribute(mcp, key::kRotationAroundUpVector,
                         rotatesAroundUpVector());
    io::setBoolAttribute(mcp, key::kZoomOnPivot, zoomsOnPivotPoint());
    mcp.appendChild(sceneUpVector().domElement(key::kSceneupVector, doc));

    auto elem = ManipulatedFrame::domElement(name, doc);
    elem.appendChild(mcp);

    return elem;
}

void ManipulatedCameraFrame::initFromDOMElement(const QDomElement &element)
{
    // No need to initialize, since default sceneUpVector and flySpeed are not
    // meaningful. It's better to keep current ones. And it would destroy
    // constraint() and referenceFrame(). *this = ManipulatedCameraFrame();
    ManipulatedFrame::initFromDOMElement(element);

    auto child = element.firstChild().toElement();
    while (!child.isNull()) {
        if (child.tagName() == key::kParameters) {
            setFlySpeed(io::qrealFromDom(child, key::kFlySpeed, flySpeed()));
            setRotatesAroundUpVector(
                io::boolFromDom(child, key::kRotationAroundUpVector, false));
            setZoomsOnPivotPoint(
                io::boolFromDom(child, key::kZoomOnPivot, false));

            auto child2 = child.firstChild().toElement();
            while (!child2.isNull()) {
                if (child2.tagName() == key::kSceneupVector) {
                    setSceneUpVector(Vec(child2));
                }

                child2 = child2.nextSibling().toElement();
            }
        }
        child = child.nextSibling().toElement();
    }
}

void ManipulatedCameraFrame::startAction(int ma, bool withConstraint)
{
    ManipulatedFrame::startAction(ma, withConstraint);
    switch (action_) {
        case QtOpenGLViewer::MoveForward:
        case QtOpenGLViewer::MoveBackward:
        case QtOpenGLViewer::Drive:
            flyTimer_.setSingleShot(false);
            flyTimer_.start(10);
            break;
        case QtOpenGLViewer::Rotate:
            constrainedRotationIsReversed_ =
                transformOf(sceneUpVector_).y < 0.0;
            break;
        default:
            break;
    }
}

void ManipulatedCameraFrame::zoom(qreal delta, const Camera *const camera)
{
    const qreal sceneRadius = camera->sceneRadius();
    if (zoomsOnPivotPoint_) {
        Vec direction = position() - camera->pivotPoint();
        if (direction.norm() > 0.02 * sceneRadius || delta > 0.0)
            translate(delta * direction);
    }
    else {
        const qreal coef = qMax(
            std::abs((camera->frame()->coordinatesOf(camera->pivotPoint())).z),
            qreal(0.2) * sceneRadius);
        Vec trans(0.0, 0.0, -coef * delta);
        translate(inverseTransformOf(trans));
    }
}

void ManipulatedCameraFrame::mouseMoveEvent(QMouseEvent *const event,
                                            Camera *const camera)
{
    // #CONNECTION# QGLViewer::mouseMoveEvent does the update().
    switch (action_) {
        case QtOpenGLViewer::Translate: {
            const QPoint delta = prevPos_ - event->pos();
            Vec trans(delta.x(), -delta.y(), 0.0);
            // Scale to fit the screen mouse displacement
            switch (camera->type()) {
                case Camera::Type::Perspective:
                    trans *=
                        2.0 * tan(camera->fieldOfView() / 2.0) *
                        std::abs(
                            (camera->frame()->coordinatesOf(pivotPoint())).z) /
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
            translate(inverseTransformOf(translationSensitivity() * trans));
            break;
        }

        case QtOpenGLViewer::MoveForward: {
            Quaternion rot = pitchYawQuaternion(event->position().x(),
                                                event->position().y(), camera);

            rotate(rot);
            // #CONNECTION# wheelEvent MOVE_FORWARD case
            //  actual translation is made in flyUpdate().
            //  translate(inverseTransformOf(Vec(0.0, 0.0, -flySpeed())));
            break;
        }

        case QtOpenGLViewer::MoveBackward: {
            Quaternion rot = pitchYawQuaternion(event->position().x(),
                                                event->position().y(), camera);

            rotate(rot);
            // actual translation is made in flyUpdate().
            // translate(inverseTransformOf(Vec(0.0, 0.0, flySpeed())));
            break;
        }

        case QtOpenGLViewer::Drive: {
            Quaternion rot = turnQuaternion(event->position().x(), camera);

            rotate(rot);
            // actual translation is made in flyUpdate().

            driveSpeed_ = 0.01 * (event->position().y() - pressPos_.y());

            break;
        }

        case QtOpenGLViewer::Zoom: {
            zoom(deltaWithPrevPos(event, camera), camera);
            break;
        }

        case QtOpenGLViewer::LookForward: {
            Quaternion rot = pitchYawQuaternion(event->position().x(),
                                                event->position().y(), camera);

            rotate(rot);
            break;
        }

        case QtOpenGLViewer::Rotate: {
            Quaternion rot;
            if (rotatesAroundUpVector_) {
                // Multiply by 2.0 to get on average about the same speed as
                // with the deformed ball

                qreal dx = 2.0 * rotationSensitivity() *
                           (prevPos_.x() - event->position().x()) /
                           camera->screenWidth();
                qreal dy = 2.0 * rotationSensitivity() *
                           (prevPos_.y() - event->position().y()) /
                           camera->screenHeight();

                if (constrainedRotationIsReversed_)
                    dx = -dx;
                Vec verticalAxis = transformOf(sceneUpVector_);
                rot = Quaternion(verticalAxis, dx) *
                      Quaternion(Vec(1.0, 0.0, 0.0), dy);
            }
            else {
                Vec trans = camera->projectedCoordinatesOf(pivotPoint());

                rot = deformedBallQuaternion(event->position().x(),
                                             event->position().y(), trans[0],
                                             trans[1], camera);
            }
            // #CONNECTION# These two methods should go together (spinning
            // detection and
            //  activation)
            computeMouseSpeed(event);
            setSpinningQuaternion(rot);
            spin();
            break;
        }

        case QtOpenGLViewer::ScreenRotate: {
            Vec trans = camera->projectedCoordinatesOf(pivotPoint());

            const qreal angle =
                atan2(event->position().y() - trans[1],
                      event->position().x() - trans[0]) -
                atan2(prevPos_.y() - trans[1], prevPos_.x() - trans[0]);

            Quaternion rot(Vec(0.0, 0.0, 1.0), angle);
            // #CONNECTION# These two methods should go together (spinning
            // detection and
            //  activation)
            computeMouseSpeed(event);
            setSpinningQuaternion(rot);
            spin();
            updateSceneUpVector();
            break;
        }

        case QtOpenGLViewer::Roll: {
            const qreal angle = M_PI * (event->position().x() - prevPos_.x()) /
                                camera->screenWidth();

            Quaternion rot(Vec(0.0, 0.0, 1.0), angle);
            rotate(rot);
            setSpinningQuaternion(rot);
            updateSceneUpVector();
            break;
        }

        case QtOpenGLViewer::ScreenTranslate: {
            Vec trans;
            int dir = mouseOriginalDirection(event);
            if (dir == 1) {
                trans.setValue(prevPos_.x() - event->position().x(), 0.0, 0.0);
            }
            else if (dir == -1) {
                trans.setValue(0.0, event->position().y() - prevPos_.y(), 0.0);
            }

            switch (camera->type()) {
                case Camera::Type::Perspective:
                    trans *=
                        2.0 * tan(camera->fieldOfView() / 2.0) *
                        std::abs(
                            (camera->frame()->coordinatesOf(pivotPoint())).z) /
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

            translate(inverseTransformOf(translationSensitivity() * trans));
            break;
        }
        case QtOpenGLViewer::ZoomOnRegion:
        case QtOpenGLViewer::NoMouseAction:
            break;
    }

    if (action_ != QtOpenGLViewer::NoMouseAction) {
        prevPos_ = event->pos();
        if (action_ != QtOpenGLViewer::ZoomOnRegion)
            // ZOOM_ON_REGION should not emit manipulated().
            // prevPos_ is used to draw rectangle feedback.
            emit manipulated();
    }
}

void ManipulatedCameraFrame::mouseReleaseEvent(QMouseEvent *const event,
                                               Camera *const camera)
{
    if ((action_ == QtOpenGLViewer::MoveForward) ||
        (action_ == QtOpenGLViewer::MoveBackward) ||
        (action_ == QtOpenGLViewer::Drive)) {
        flyTimer_.stop();
    }

    if (action_ == QtOpenGLViewer::ZoomOnRegion) {
        camera->fitScreenRegion(QRect(pressPos_, event->pos()));
    }

    ManipulatedFrame::mouseReleaseEvent(event, camera);
}

void ManipulatedCameraFrame::wheelEvent(QWheelEvent *const event,
                                        Camera *const camera)
{
    // #CONNECTION# QGLViewer::setWheelBinding, ManipulatedFrame::wheelEvent.
    switch (action_) {
        case QtOpenGLViewer::Zoom: {
            zoom(wheelDelta(event), camera);
            emit manipulated();
            break;
        }
        case QtOpenGLViewer::MoveForward:
        case QtOpenGLViewer::MoveBackward:
            // #CONNECTION# mouseMoveEvent() MOVE_FORWARD case
            translate(inverseTransformOf(
                Vec(0.0, 0.0, 0.2 * flySpeed() * event->angleDelta().y())));
            emit manipulated();
            break;
        default:
            break;
    }

    // #CONNECTION# startAction should always be called before
    if (previousConstraint_)
        setConstraint(previousConstraint_);

    // The wheel triggers a fastDraw. A final update() is needed after the last
    // wheel event to polish the rendering using draw(). Since the last wheel
    // event does not say its name, we use the flyTimer_ to trigger flyUpdate(),
    // which emits manipulated. Two wheel events separated by more than this
    // delay milliseconds will trigger a draw().
    constexpr int finalDrawAfterWheelEventDelay = 400;

    // Starts (or prolungates) the timer.
    flyTimer_.setSingleShot(true);
    flyTimer_.start(finalDrawAfterWheelEventDelay);

    // This could also be done *before* manipulated is emitted, so that
    // isManipulated() returns false. But then fastDraw would not be used with
    // wheel. Detecting the last wheel event and forcing a final draw() is done
    // using the timer_.
    action_ = QtOpenGLViewer::NoMouseAction;
}

Quaternion ManipulatedCameraFrame::turnQuaternion(int x,
                                                  const Camera *const camera)
{
    return {Vec{0., 1., 0.},
            rotationSensitivity() * (prevPos_.x() - x) / camera->screenWidth()};
}

Quaternion ManipulatedCameraFrame::pitchYawQuaternion(
    int x, int y, const Camera *const camera)
{
    const Quaternion rotX{
        Vec{1., 0., 0.},
        rotationSensitivity() * (prevPos_.y() - y) / camera->screenHeight()};
    const Quaternion rotY{
        transformOf(sceneUpVector()),
        rotationSensitivity() * (prevPos_.x() - x) / camera->screenWidth()};
    return rotY * rotX;
}
