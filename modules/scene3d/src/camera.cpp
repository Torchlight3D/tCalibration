#include "camera.h"

#include <GL/glu.h>

#include "dom_utils.h"
#include "key_frame_interpolator.h"
#include "manipulated_camera_frame.h"

using namespace viewer;

namespace {

constexpr double kDefaultIoDistance = 0.062;
constexpr double kDefaultPhysicalScreenWidth = 0.5;

// Compute a Matrix3d determinant.
inline qreal det(qreal m00, qreal m01, qreal m02, qreal m10, qreal m11,
                 qreal m12, qreal m20, qreal m21, qreal m22)
{
    return m00 * m11 * m22 + m01 * m12 * m20 + m02 * m10 * m21 -
           m20 * m11 * m02 - m10 * m01 * m22 - m00 * m21 * m12;
}

// Computes the index of element [i][j] in a \c qreal matrix[3][4].
inline unsigned int ind(unsigned int i, unsigned int j) { return (i * 4 + j); }

} // namespace

Camera::Camera()
    : frame_(nullptr),
      fieldOfView_(M_PI / 4.0),
      modelViewMatrixIsUpToDate_(false),
      projectionMatrixIsUpToDate_(false)
{
    // #CONNECTION# Camera copy constructor
    interpolationKfi_ = new KeyFrameInterpolator;
    // Requires the interpolationKfi_
    setFrame(new ManipulatedCameraFrame());

    // #CONNECTION# All these default values identical in initFromDOMElement.

    // Requires fieldOfView() to define focusDistance()
    setSceneRadius(1.0);

    // Initial value (only scaled after this)
    orthoCoef_ = tan(fieldOfView() / 2.0);

    // Also defines the pivotPoint(), which changes orthoCoef_. Requires a
    // frame().
    setSceneCenter(Vec(0.0, 0.0, 0.0));

    // Requires fieldOfView() when called with ORTHOGRAPHIC. Attention to
    // projectionMatrix_ below.
    setType(Type::Perspective);

    // #CONNECTION# initFromDOMElement default values
    setZNearCoefficient(0.005);
    setZClippingCoefficient(sqrt(3.0));

    // Dummy values
    setScreenWidthAndHeight(600, 400);

    // Stereo parameters
    setIODistance(kDefaultIoDistance);
    setPhysicalScreenWidth(kDefaultPhysicalScreenWidth);
    // focusDistance is set from setFieldOfView()

    // Default value
    setDevicePixelRatio(1.0);

    // #CONNECTION# Camera copy constructor
    for (int j{0}; j < 16; ++j) {
        modelViewMatrix_[j] = ((j % 5 == 0) ? 1.0 : 0.0);
        // #CONNECTION# computeProjectionMatrix() is lazy and assumes 0.0 almost
        // everywhere.
        projectionMatrix_[j] = 0.0;
    }
    computeProjectionMatrix();
}

Camera::~Camera()
{
    delete frame_;
    delete interpolationKfi_;
}

Camera::Camera(const Camera &camera) : QObject(), frame_(nullptr)
{
    interpolationKfi_ = new KeyFrameInterpolator;
    // Requires the interpolationKfi_
    setFrame(new ManipulatedCameraFrame(*camera.frame()));

    for (int j{0}; j < 16; ++j) {
        modelViewMatrix_[j] = ((j % 5 == 0) ? 1.0 : 0.0);
        // #CONNECTION# computeProjectionMatrix() is lazy and assumes 0.0 almost
        // everywhere.
        projectionMatrix_[j] = 0.0;
    }

    (*this) = camera;
}

Camera &Camera::operator=(const Camera &camera)
{
    setScreenWidthAndHeight(camera.screenWidth(), camera.screenHeight());
    setFieldOfView(camera.fieldOfView());
    setSceneRadius(camera.sceneRadius());
    setSceneCenter(camera.sceneCenter());
    setZNearCoefficient(camera.zNearCoefficient());
    setZClippingCoefficient(camera.zClippingCoefficient());
    setType(camera.type());

    // Stereo parameters
    setIODistance(camera.IODistance());
    setFocusDistance(camera.focusDistance());
    setPhysicalScreenWidth(camera.physicalScreenWidth());

    orthoCoef_ = camera.orthoCoef_;
    projectionMatrixIsUpToDate_ = false;

    // frame_ and interpolationKfi_ pointers are not shared.
    frame_->setReferenceFrame(nullptr);
    frame_->setPosition(camera.position());
    frame_->setOrientation(camera.orientation());

    interpolationKfi_->resetInterpolation();

    kfi_ = camera.kfi_;

    computeProjectionMatrix();
    computeModelViewMatrix();

    return *this;
}

Camera::Type Camera::type() const { return type_; }

void Camera::setType(Type type)
{
    // make ORTHOGRAPHIC frustum fit PERSPECTIVE (at least in plane normal to
    // viewDirection(), passing through RAP). Done only when CHANGING type since
    // orthoCoef_ may have been changed with a setPivotPoint() in the meantime.
    if ((type == Type::Orthographic) && (type_ == Type::Perspective)) {
        orthoCoef_ = tan(fieldOfView() / 2.0);
    }
    type_ = type;
    projectionMatrixIsUpToDate_ = false;
}

qreal Camera::fieldOfView() const { return fieldOfView_; }

void Camera::setFieldOfView(qreal fov)
{
    fieldOfView_ = fov;
    setFocusDistance(sceneRadius() / tan(fov / 2.0));
    projectionMatrixIsUpToDate_ = false;
}

qreal Camera::horizontalFieldOfView() const
{
    return 2.0 * atan(tan(fieldOfView() / 2.0) * aspectRatio());
}

qreal Camera::aspectRatio() const
{
    return screenWidth_ / static_cast<qreal>(screenHeight_);
}

int Camera::screenWidth() const { return screenWidth_; }

int Camera::screenHeight() const { return screenHeight_; }

qreal Camera::devicePixelRatio() const { return devicePixelRatio_; }

qreal Camera::zNearCoefficient() const { return zNearCoef_; }

qreal Camera::zClippingCoefficient() const { return zClippingCoef_; }

void Camera::setScreenWidthAndHeight(int width, int height)
{
    // Prevent negative and zero dimensions that would cause divisions by zero.
    screenWidth_ = width > 0 ? width : 1;
    screenHeight_ = height > 0 ? height : 1;
    projectionMatrixIsUpToDate_ = false;
}

void Camera::setDevicePixelRatio(qreal ratio) { devicePixelRatio_ = ratio; }

qreal Camera::sceneRadius() const { return sceneRadius_; }

Vec Camera::sceneCenter() const { return sceneCenter_; }

qreal Camera::zNear() const
{
    const qreal zNearScene = zClippingCoefficient() * sceneRadius();
    qreal z = distanceToSceneCenter() - zNearScene;

    // Prevents negative or null zNear values.
    const qreal zMin = zNearCoefficient() * zNearScene;
    if (z < zMin)
        switch (type()) {
            case Type::Perspective:
                z = zMin;
                break;
            case Type::Orthographic:
                z = 0.0;
                break;
            default:
                break;
        }
    return z;
}

qreal Camera::zFar() const
{
    return distanceToSceneCenter() + zClippingCoefficient() * sceneRadius();
}

ManipulatedCameraFrame *Camera::frame() const { return frame_; }

void Camera::setFrame(ManipulatedCameraFrame *const mcf)
{
    if (!mcf) {
        return;
    }

    if (frame_) {
        disconnect(frame_, &Frame::modified, this, &Camera::onFrameModified);
    }

    frame_ = mcf;
    interpolationKfi_->setFrame(frame());

    connect(frame_, &Frame::modified, this, &Camera::onFrameModified);
    onFrameModified();
}

qreal Camera::distanceToSceneCenter() const
{
    return fabs((frame()->coordinatesOf(sceneCenter())).z);
}

void Camera::getOrthoWidthHeight(GLdouble &halfWidth,
                                 GLdouble &halfHeight) const
{
    const qreal dist = orthoCoef_ * fabs(cameraCoordinatesOf(pivotPoint()).z);
    // fitScreenRegion
    halfWidth = dist * ((aspectRatio() < 1.0) ? 1.0 : aspectRatio());
    halfHeight = dist * ((aspectRatio() < 1.0) ? 1.0 / aspectRatio() : 1.0);
}

void Camera::computeProjectionMatrix() const
{
    if (projectionMatrixIsUpToDate_)
        return;

    const qreal ZNear = zNear();
    const qreal ZFar = zFar();

    switch (type()) {
        case Type::Perspective: {
            // all non null coefficients were set to 0.0 in constructor.
            const qreal f = 1.0 / tan(fieldOfView() / 2.0);
            projectionMatrix_[0] = f / aspectRatio();
            projectionMatrix_[5] = f;
            projectionMatrix_[10] = (ZNear + ZFar) / (ZNear - ZFar);
            projectionMatrix_[11] = -1.0;
            projectionMatrix_[14] = 2.0 * ZNear * ZFar / (ZNear - ZFar);
            projectionMatrix_[15] = 0.0;
            // same as gluPerspective( 180.0*fieldOfView()/M_PI, aspectRatio(),
            // zNear(), zFar() );
            break;
        }
        case Type::Orthographic: {
            GLdouble w, h;
            getOrthoWidthHeight(w, h);
            projectionMatrix_[0] = 1.0 / w;
            projectionMatrix_[5] = 1.0 / h;
            projectionMatrix_[10] = -2.0 / (ZFar - ZNear);
            projectionMatrix_[11] = 0.0;
            projectionMatrix_[14] = -(ZFar + ZNear) / (ZFar - ZNear);
            projectionMatrix_[15] = 1.0;
            // same as glOrtho( -w, w, -h, h, zNear(), zFar() );
            break;
        }
    }

    projectionMatrixIsUpToDate_ = true;
}

void Camera::computeModelViewMatrix() const
{
    if (modelViewMatrixIsUpToDate_) {
        return;
    }

    const Quaternion q = frame()->orientation();

    const qreal q00 = 2.0 * q[0] * q[0];
    const qreal q11 = 2.0 * q[1] * q[1];
    const qreal q22 = 2.0 * q[2] * q[2];

    const qreal q01 = 2.0 * q[0] * q[1];
    const qreal q02 = 2.0 * q[0] * q[2];
    const qreal q03 = 2.0 * q[0] * q[3];

    const qreal q12 = 2.0 * q[1] * q[2];
    const qreal q13 = 2.0 * q[1] * q[3];

    const qreal q23 = 2.0 * q[2] * q[3];

    modelViewMatrix_[0] = 1.0 - q11 - q22;
    modelViewMatrix_[1] = q01 - q23;
    modelViewMatrix_[2] = q02 + q13;
    modelViewMatrix_[3] = 0.0;

    modelViewMatrix_[4] = q01 + q23;
    modelViewMatrix_[5] = 1.0 - q22 - q00;
    modelViewMatrix_[6] = q12 - q03;
    modelViewMatrix_[7] = 0.0;

    modelViewMatrix_[8] = q02 - q13;
    modelViewMatrix_[9] = q12 + q03;
    modelViewMatrix_[10] = 1.0 - q11 - q00;
    modelViewMatrix_[11] = 0.0;

    const Vec t = q.inverseRotate(frame()->position());

    modelViewMatrix_[12] = -t.x;
    modelViewMatrix_[13] = -t.y;
    modelViewMatrix_[14] = -t.z;
    modelViewMatrix_[15] = 1.0;

    modelViewMatrixIsUpToDate_ = true;
}

void Camera::loadProjectionMatrix(bool reset) const
{
    // WARNING: makeCurrent must be called by every calling method
    glMatrixMode(GL_PROJECTION);

    if (reset) {
        glLoadIdentity();
    }

    computeProjectionMatrix();

    glMultMatrixd(projectionMatrix_);
}

void Camera::loadModelViewMatrix(bool reset) const
{
    // WARNING: makeCurrent must be called by every calling method
    glMatrixMode(GL_MODELVIEW);
    computeModelViewMatrix();
    if (reset) {
        glLoadMatrixd(modelViewMatrix_);
    }
    else {
        glMultMatrixd(modelViewMatrix_);
    }
}

void Camera::loadProjectionMatrixStereo(bool leftBuffer) const
{
    qreal left, right, bottom, top;
    qreal screenHalfWidth, halfWidth, side, shift, delta;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    switch (type()) {
        case Type::Perspective:
            // compute half width of screen,
            // corresponding to zero parallax plane to deduce decay of cameras
            screenHalfWidth =
                focusDistance() * tan(horizontalFieldOfView() / 2.0);
            shift = screenHalfWidth * IODistance() / physicalScreenWidth();
            // should be * current y  / y total
            // to take into account that the window doesn't cover the entire
            // screen

            // compute half width of "view" at znear and the delta corresponding
            // to the shifted camera to deduce what to set for asymmetric
            // frustums
            halfWidth = zNear() * tan(horizontalFieldOfView() / 2.0);
            delta = shift * zNear() / focusDistance();
            side = leftBuffer ? -1.0 : 1.0;

            left = -halfWidth + side * delta;
            right = halfWidth + side * delta;
            top = halfWidth / aspectRatio();
            bottom = -top;
            glFrustum(left, right, bottom, top, zNear(), zFar());
            break;
        case Type::Orthographic:
            qWarning() << "Stereo is not available with Orthographic mode";
            break;
        default:
            break;
    }
}

void Camera::loadModelViewMatrixStereo(bool leftBuffer) const
{
    // WARNING: makeCurrent must be called by every calling method
    glMatrixMode(GL_MODELVIEW);

    qreal halfWidth = focusDistance() * tan(horizontalFieldOfView() / 2.0);
    qreal shift =
        halfWidth * IODistance() /
        physicalScreenWidth(); // * current window width / full screen width

    computeModelViewMatrix();
    if (leftBuffer) {
        modelViewMatrix_[12] -= shift;
    }
    else {
        modelViewMatrix_[12] += shift;
    }
    glLoadMatrixd(modelViewMatrix_);
}

void Camera::getProjectionMatrix(GLdouble m[16]) const
{
    computeProjectionMatrix();
    for (int i{0}; i < 16; ++i) {
        m[i] = projectionMatrix_[i];
    }
}

void Camera::getProjectionMatrix(GLfloat m[16]) const
{
    static GLdouble mat[16];
    getProjectionMatrix(mat);
    for (int i{0}; i < 16; ++i) {
        m[i] = float(mat[i]);
    }
}

void Camera::getModelViewMatrix(GLdouble m[16]) const
{
    // May not be needed, but easier like this.
    // Prevents from retrieving matrix in stereo mode -> overwrites shifted
    // value.
    computeModelViewMatrix();
    for (int i{0}; i < 16; ++i) {
        m[i] = modelViewMatrix_[i];
    }
}

void Camera::getModelViewMatrix(GLfloat m[16]) const
{
    static GLdouble mat[16];
    getModelViewMatrix(mat);
    for (int i{0}; i < 16; ++i) {
        m[i] = float(mat[i]);
    }
}

void Camera::getModelViewProjectionMatrix(GLdouble m[16]) const
{
    GLdouble mv[16];
    GLdouble proj[16];
    getModelViewMatrix(mv);
    getProjectionMatrix(proj);

    for (int i{0}; i < 4; ++i) {
        for (int j{0}; j < 4; ++j) {
            qreal sum{0.};
            for (int k{0}; k < 4; ++k) {
                sum += proj[i + 4 * k] * mv[k + 4 * j];
            }
            m[i + 4 * j] = sum;
        }
    }
}

void Camera::getModelViewProjectionMatrix(GLfloat m[16]) const
{
    static GLdouble mat[16];
    getModelViewProjectionMatrix(mat);
    for (int i{0}; i < 16; ++i) {
        m[i] = float(mat[i]);
    }
}

void Camera::setSceneRadius(qreal radius)
{
    if (radius <= 0.0) {
        qWarning("Scene radius must be positive - Ignoring value");
        return;
    }

    sceneRadius_ = radius;
    projectionMatrixIsUpToDate_ = false;

    setFocusDistance(sceneRadius() / tan(fieldOfView() / 2.0));

    frame()->setFlySpeed(0.01 * sceneRadius());
}

void Camera::setSceneBoundingBox(const Vec &min, const Vec &max)
{
    setSceneCenter((min + max) / 2.0);
    setSceneRadius(0.5 * (max - min).norm());
}

void Camera::setSceneCenter(const Vec &center)
{
    sceneCenter_ = center;
    setPivotPoint(sceneCenter());
    projectionMatrixIsUpToDate_ = false;
}

bool Camera::setSceneCenterFromPixel(const QPoint &pixel)
{
    bool found;
    const Vec point = pointUnderPixel(pixel, found);

    if (found) {
        setSceneCenter(point);
    }
    return found;
}

void Camera::setPivotPoint(const Vec &point)
{
    const qreal prevDist = fabs(cameraCoordinatesOf(pivotPoint()).z);

    // If frame's RAP is set directly, projectionMatrixIsUpToDate_ should also
    // be set to false to ensure proper recomputation of the ORTHO projection
    // matrix.
    frame()->setPivotPoint(point);

    // orthoCoef_ is used to compensate for changes of the pivotPoint, so that
    // the image does not change when the pivotPoint is changed in ORTHOGRAPHIC
    // mode.
    const qreal newDist = fabs(cameraCoordinatesOf(pivotPoint()).z);
    // Prevents division by zero when rap is set to camera position
    if ((prevDist > 1e-9) && (newDist > 1e-9)) {
        orthoCoef_ *= prevDist / newDist;
    }
    projectionMatrixIsUpToDate_ = false;
}

bool Camera::setPivotPointFromPixel(const QPoint &pixel)
{
    bool found;
    const Vec point = pointUnderPixel(pixel, found);
    if (found) {
        setPivotPoint(point);
    }
    return found;
}

qreal Camera::pixelGLRatio(const Vec &position) const
{
    switch (type()) {
        case Type::Perspective:
            return 2.0 * fabs((frame()->coordinatesOf(position)).z) *
                   tan(fieldOfView() / 2.0) / screenHeight();
        case Type::Orthographic: {
            GLdouble w, h;
            getOrthoWidthHeight(w, h);
            return 2.0 * h / screenHeight();
        }
        default:
            break;
    }

    return 1.0;
}

void Camera::setFOVToFitScene()
{
    if (distanceToSceneCenter() > sqrt(2.0) * sceneRadius())
        setFieldOfView(2.0 * asin(sceneRadius() / distanceToSceneCenter()));
    else
        setFieldOfView(M_PI / 2.0);
}

void Camera::interpolateToZoomOnPixel(const QPoint &pixel)
{
    const qreal coef{0.1};

    bool found;
    Vec target = pointUnderPixel(pixel, found);

    if (!found) {
        return;
    }

    if (interpolationKfi_->interpolationIsStarted()) {
        interpolationKfi_->stopInterpolation();
    }

    interpolationKfi_->deletePath();
    interpolationKfi_->addKeyFrame(*(frame()));

    interpolationKfi_->addKeyFrame(
        Frame(0.3 * frame()->position() + 0.7 * target, frame()->orientation()),
        0.4);

    // Small hack: attach a temporary frame to take advantage of lookAt without
    // modifying frame
    static ManipulatedCameraFrame *tempFrame = new ManipulatedCameraFrame();
    ManipulatedCameraFrame *const originalFrame = frame();
    tempFrame->setPosition(coef * frame()->position() + (1.0 - coef) * target);
    tempFrame->setOrientation(frame()->orientation());
    setFrame(tempFrame);
    lookAt(target);
    setFrame(originalFrame);

    interpolationKfi_->addKeyFrame(*(tempFrame), 1.0);

    interpolationKfi_->startInterpolation();
}

void Camera::interpolateToFitScene()
{
    if (interpolationKfi_->interpolationIsStarted()) {
        interpolationKfi_->stopInterpolation();
    }

    interpolationKfi_->deletePath();
    interpolationKfi_->addKeyFrame(*(frame()));

    // Small hack:  attach a temporary frame to take advantage of lookAt without
    // modifying frame
    static ManipulatedCameraFrame *tempFrame = new ManipulatedCameraFrame();
    ManipulatedCameraFrame *const originalFrame = frame();
    tempFrame->setPosition(frame()->position());
    tempFrame->setOrientation(frame()->orientation());
    setFrame(tempFrame);
    showEntireScene();
    setFrame(originalFrame);

    interpolationKfi_->addKeyFrame(*(tempFrame));

    interpolationKfi_->startInterpolation();
}

void Camera::interpolateTo(const Frame &fr, qreal duration)
{
    if (interpolationKfi_->interpolationIsStarted()) {
        interpolationKfi_->stopInterpolation();
    }

    interpolationKfi_->deletePath();
    interpolationKfi_->addKeyFrame(*(frame()));
    interpolationKfi_->addKeyFrame(fr, duration);

    interpolationKfi_->startInterpolation();
}

Vec Camera::pointUnderPixel(const QPoint &pixel, bool &found) const
{
    float depth;
    // Qt uses upper corner for its origin while GL uses the lower corner.
    glReadPixels(pixel.x() * devicePixelRatio_,
                 devicePixelRatio_ * (screenHeight() - pixel.y()) - 1, 1, 1,
                 GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
    found = static_cast<double>(depth) < 1.0;
    Vec point(pixel.x(), pixel.y(), static_cast<double>(depth));
    point = unprojectedCoordinatesOf(point);
    return point;
}

void Camera::showEntireScene() { fitSphere(sceneCenter(), sceneRadius()); }

void Camera::centerScene()
{
    frame()->projectOnLine(sceneCenter(), viewDirection());
}

void Camera::lookAt(const Vec &target)
{
    setViewDirection(target - position());
}

void Camera::fitSphere(const Vec &center, qreal radius)
{
    qreal distance{0.};
    switch (type()) {
        case Type::Perspective: {
            const qreal yview = radius / sin(fieldOfView() / 2.);
            const qreal xview = radius / sin(horizontalFieldOfView() / 2.);
            distance = qMax(xview, yview);
            break;
        }
        case Type::Orthographic: {
            distance = ((center - pivotPoint()) * viewDirection()) +
                       (radius / orthoCoef_);
            break;
        }
        default:
            break;
    }

    Vec newPos(center - distance * viewDirection());
    frame()->setPositionWithConstraint(newPos);
}

void Camera::fitBoundingBox(const Vec &min, const Vec &max)
{
    qreal diameter = qMax(fabs(max[1] - min[1]), fabs(max[0] - min[0]));
    diameter = qMax(fabs(max[2] - min[2]), diameter);
    fitSphere(0.5 * (min + max), 0.5 * diameter);
}

void Camera::fitScreenRegion(const QRect &rectangle)
{
    const Vec vd = viewDirection();
    const qreal distToPlane = distanceToSceneCenter();
    const QPoint center = rectangle.center();

    Vec orig, dir;
    convertClickToLine(center, orig, dir);
    Vec newCenter = orig + distToPlane / (dir * vd) * dir;

    convertClickToLine(QPoint(rectangle.x(), center.y()), orig, dir);
    const Vec pointX = orig + distToPlane / (dir * vd) * dir;

    convertClickToLine(QPoint(center.x(), rectangle.y()), orig, dir);
    const Vec pointY = orig + distToPlane / (dir * vd) * dir;

    qreal distance = 0.0;
    switch (type()) {
        case Type::Perspective: {
            const qreal distX = (pointX - newCenter).norm() /
                                sin(horizontalFieldOfView() / 2.0);
            const qreal distY =
                (pointY - newCenter).norm() / sin(fieldOfView() / 2.0);
            distance = qMax(distX, distY);
            break;
        }
        case Type::Orthographic: {
            const qreal dist = ((newCenter - pivotPoint()) * vd);
            const qreal distX = (pointX - newCenter).norm() / orthoCoef_ /
                                ((aspectRatio() < 1.0) ? 1.0 : aspectRatio());
            const qreal distY =
                (pointY - newCenter).norm() / orthoCoef_ /
                ((aspectRatio() < 1.0) ? 1.0 / aspectRatio() : 1.0);
            distance = dist + qMax(distX, distY);
            break;
        }
        default:
            break;
    }

    Vec newPos(newCenter - distance * vd);
    frame()->setPositionWithConstraint(newPos);
}

void Camera::setUpVector(const Vec &up, bool noMove)
{
    Quaternion q(Vec(0.0, 1.0, 0.0), frame()->transformOf(up));

    if (!noMove) {
        frame()->setPosition(pivotPoint() -
                             (frame()->orientation() * q)
                                 .rotate(frame()->coordinatesOf(pivotPoint())));
    }

    frame()->rotate(q);

    // Useful in fly mode to keep the horizontal direction.
    frame()->updateSceneUpVector();
}

Quaternion Camera::orientation() const { return frame()->orientation(); }

void Camera::setOrientation(const Quaternion &q)
{
    frame()->setOrientation(q);
    frame()->updateSceneUpVector();
}

void Camera::setOrientation(qreal theta, qreal phi)
{
    const Quaternion rot1{Vec{0., 1., 0.}, theta};
    const Quaternion rot2{Vec{-cos(theta), 0., sin(theta)}, phi};
    setOrientation(rot1 * rot2);
}

Vec Camera::viewDirection() const
{
    return frame()->inverseTransformOf(Vec(0., 0., -1.));
}

void Camera::setViewDirection(const Vec &direction)
{
    if (direction.squaredNorm() < 1e-10) {
        return;
    }

    Vec xAxis = direction ^ upVector();
    if (xAxis.squaredNorm() < 1e-10) {
        // target is aligned with upVector, this means a rotation around X axis
        // X axis is then unchanged, let's keep it !
        xAxis = frame()->inverseTransformOf(Vec(1.0, 0.0, 0.0));
    }

    Quaternion q;
    q.setFromRotatedBasis(xAxis, xAxis ^ direction, -direction);
    frame()->setOrientationWithConstraint(q);
}

Vec Camera::position() const { return frame()->position(); }

void Camera::setPosition(const Vec &pos) { frame()->setPosition(pos); }

Vec Camera::upVector() const
{
    return frame()->inverseTransformOf(Vec(0.0, 1.0, 0.0));
}

Vec Camera::rightVector() const
{
    return frame()->inverseTransformOf(Vec(1.0, 0.0, 0.0));
}

Vec Camera::cameraCoordinatesOf(const Vec &src) const
{
    return frame()->coordinatesOf(src);
}

Vec Camera::worldCoordinatesOf(const Vec &src) const
{
    return frame()->inverseCoordinatesOf(src);
}

qreal Camera::flySpeed() const { return frame()->flySpeed(); }

void Camera::setFlySpeed(qreal speed) { frame()->setFlySpeed(speed); }

Vec Camera::pivotPoint() const { return frame()->pivotPoint(); }

void Camera::setFromModelViewMatrix(const GLdouble *const modelViewMatrix)
{
    // Get upper left (rotation) matrix
    qreal upperLeft[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            upperLeft[i][j] = modelViewMatrix[i * 4 + j];
        }
    }

    // Transform upperLeft into the associated Quaternion
    Quaternion q;
    q.setFromRotationMatrix(upperLeft);

    setOrientation(q);
    setPosition(-q.rotate(
        Vec(modelViewMatrix[12], modelViewMatrix[13], modelViewMatrix[14])));
}

void Camera::setFromProjectionMatrix(const qreal matrix[12])
{
    // The 3 lines of the matrix are the normals to the planes x=0, y=0, z=0
    // in the camera CS. As we normalize them, we do not need the 4th
    // coordinate.
    Vec line_0(matrix[ind(0, 0)], matrix[ind(0, 1)], matrix[ind(0, 2)]);
    Vec line_1(matrix[ind(1, 0)], matrix[ind(1, 1)], matrix[ind(1, 2)]);
    Vec line_2(matrix[ind(2, 0)], matrix[ind(2, 1)], matrix[ind(2, 2)]);

    line_0.normalize();
    line_1.normalize();
    line_2.normalize();

    // The camera position is at (0,0,0) in the camera CS so it is the
    // intersection of the 3 planes. It can be seen as the kernel
    // of the 3x4 projection matrix. We calculate it through 4 dimensional
    // vectorial product. We go directly into 3D that is to say we directly
    // divide the first 3 coordinates by the 4th one.

    // We derive the 4 dimensional vectorial product formula from the
    // computation of a 4x4 determinant that is developped according to
    // its 4th column. This implies some 3x3 determinants.
    const Vec cam_pos =
        Vec(det(matrix[ind(0, 1)], matrix[ind(0, 2)], matrix[ind(0, 3)],
                matrix[ind(1, 1)], matrix[ind(1, 2)], matrix[ind(1, 3)],
                matrix[ind(2, 1)], matrix[ind(2, 2)], matrix[ind(2, 3)]),
            -det(matrix[ind(0, 0)], matrix[ind(0, 2)], matrix[ind(0, 3)],
                 matrix[ind(1, 0)], matrix[ind(1, 2)], matrix[ind(1, 3)],
                 matrix[ind(2, 0)], matrix[ind(2, 2)], matrix[ind(2, 3)]),
            det(matrix[ind(0, 0)], matrix[ind(0, 1)], matrix[ind(0, 3)],
                matrix[ind(1, 0)], matrix[ind(1, 1)], matrix[ind(1, 3)],
                matrix[ind(2, 0)], matrix[ind(2, 1)], matrix[ind(2, 3)])) /
        (-det(matrix[ind(0, 0)], matrix[ind(0, 1)], matrix[ind(0, 2)],
              matrix[ind(1, 0)], matrix[ind(1, 1)], matrix[ind(1, 2)],
              matrix[ind(2, 0)], matrix[ind(2, 1)], matrix[ind(2, 2)]));

    // We compute the rotation matrix column by column.

    // GL Z axis is front facing.
    Vec column_2 = -line_2;

    // X-axis is almost like line_0 but should be orthogonal to the Z axis.
    Vec column_0 = ((column_2 ^ line_0) ^ column_2);
    column_0.normalize();

    // Y-axis is almost like line_1 but should be orthogonal to the Z axis.
    // Moreover line_1 is downward oriented as the screen CS.
    Vec column_1 = -((column_2 ^ line_1) ^ column_2);
    column_1.normalize();

    qreal rot[3][3];
    rot[0][0] = column_0[0];
    rot[1][0] = column_0[1];
    rot[2][0] = column_0[2];

    rot[0][1] = column_1[0];
    rot[1][1] = column_1[1];
    rot[2][1] = column_1[2];

    rot[0][2] = column_2[0];
    rot[1][2] = column_2[1];
    rot[2][2] = column_2[2];

    // We compute the field of view

    // line_1^column_0 -> vector of intersection line between
    // y_screen=0 and x_camera=0 plane.
    // column_2*(...)  -> cos of the angle between Z vector et y_screen=0 plane
    // * 2 -> field of view = 2 * half angle

    // We need some intermediate values.
    Vec dummy = line_1 ^ column_0;
    dummy.normalize();
    qreal fov = acos(column_2 * dummy) * 2.0;

    // We set the camera.
    Quaternion q;
    q.setFromRotationMatrix(rot);
    setOrientation(q);
    setPosition(cam_pos);
    setFieldOfView(fov);
}

//// persp : projectionMatrix_[0]  = f/aspectRatio();
// void Camera::setFromProjectionMatrix(const GLdouble *projectionMatrix)
//{
//     QString message;
//     if ((fabs(projectionMatrix[1]) > 1e-3) ||
//         (fabs(projectionMatrix[2]) > 1e-3) ||
//         (fabs(projectionMatrix[3]) > 1e-3) ||
//         (fabs(projectionMatrix[4]) > 1e-3) ||
//         (fabs(projectionMatrix[6]) > 1e-3) ||
//         (fabs(projectionMatrix[7]) > 1e-3) ||
//         (fabs(projectionMatrix[8]) > 1e-3) ||
//         (fabs(projectionMatrix[9]) > 1e-3))
//         message = "Non null coefficient in projection matrix - Aborting";
//     else if ((fabs(projectionMatrix[11] + 1.0) < 1e-5) &&
//              (fabs(projectionMatrix[15]) < 1e-5)) {
//         if (projectionMatrix[5] < 1e-4) {
//             message =
//                 "Negative field of view in Camera::setFromProjectionMatrix";
//         }
//         else {
//             setType(Camera::Perspective);
//         }
//     }
//     else if ((fabs(projectionMatrix[11]) < 1E-5) &&
//              (fabs(projectionMatrix[15] - 1.0) < 1E-5)) {
//         setType(Camera::Orthographic);
//     }
//     else {
//         message = "Unable to determine camera type in setFromProjectionMatrix
//         "
//                   "- Aborting ";
//     }

//    if (!message.isEmpty()) {
//        qWarning() << message;
//        return;
//    }

//    switch (type()) {
//        case Camera::Perspective: {
//            setFieldOfView(2.0 * atan(1.0 / projectionMatrix[5]));
//            const qreal far =
//                projectionMatrix[14] / (2.0 * (1.0 + projectionMatrix[10]));
//            const qreal near = (projectionMatrix[10] + 1.0) /
//                               (projectionMatrix[10] - 1.0) * far;
//            setSceneRadius((far - near) / 2.0);
//            setSceneCenter(position() +
//                           (near + sceneRadius()) * viewDirection());
//            break;
//        }
//        case Camera::Orthographic: {
//            GLdouble w, h;
//            getOrthoWidthHeight(w, h);
//            projectionMatrix_[0] = 1.0 / w;
//            projectionMatrix_[5] = 1.0 / h;
//            projectionMatrix_[10] = -2.0 / (ZFar - ZNear);
//            projectionMatrix_[11] = 0.0;
//            projectionMatrix_[14] = -(ZFar + ZNear) / (ZFar - ZNear);
//            projectionMatrix_[15] = 1.0;
//            // same as glOrtho( -w, w, -h, h, zNear(), zFar() );
//            break;
//        }
//    }
//}

void Camera::getCameraCoordinatesOf(const qreal src[3], qreal res[3]) const
{
    Vec r = cameraCoordinatesOf(Vec(src));
    for (int i = 0; i < 3; ++i) {
        res[i] = r[i];
    }
}

void Camera::getWorldCoordinatesOf(const qreal src[3], qreal res[3]) const
{
    Vec r = worldCoordinatesOf(Vec(src));
    for (int i = 0; i < 3; ++i) {
        res[i] = r[i];
    }
}

void Camera::getViewport(GLint viewport[4]) const
{
    viewport[0] = 0;
    viewport[1] = screenHeight();
    viewport[2] = screenWidth();
    viewport[3] = -screenHeight();
}

Vec Camera::projectedCoordinatesOf(const Vec &src, const Frame *frame) const
{
    GLdouble x, y, z;
    static GLint viewport[4];
    getViewport(viewport);

    if (frame) {
        const Vec tmp = frame->inverseCoordinatesOf(src);
        gluProject(tmp.x, tmp.y, tmp.z, modelViewMatrix_, projectionMatrix_,
                   viewport, &x, &y, &z);
    }
    else {
        gluProject(src.x, src.y, src.z, modelViewMatrix_, projectionMatrix_,
                   viewport, &x, &y, &z);
    }

    return Vec(x, y, z);
}

Vec Camera::unprojectedCoordinatesOf(const Vec &src, const Frame *frame) const
{
    GLdouble x, y, z;
    static GLint viewport[4];
    getViewport(viewport);
    gluUnProject(src.x, src.y, src.z, modelViewMatrix_, projectionMatrix_,
                 viewport, &x, &y, &z);
    if (frame) {
        return frame->coordinatesOf(Vec(x, y, z));
    }

    return Vec(x, y, z);
}

void Camera::getProjectedCoordinatesOf(const qreal src[3], qreal res[3],
                                       const Frame *frame) const
{
    Vec r = projectedCoordinatesOf(Vec(src), frame);
    for (int i = 0; i < 3; ++i) {
        res[i] = r[i];
    }
}

void Camera::getUnprojectedCoordinatesOf(const qreal src[3], qreal res[3],
                                         const Frame *frame) const
{
    Vec r = unprojectedCoordinatesOf(Vec(src), frame);
    for (int i = 0; i < 3; ++i) {
        res[i] = r[i];
    }
}

KeyFrameInterpolator *Camera::keyFrameInterpolator(unsigned int i) const
{
    if (kfi_.contains(i))
        return kfi_[i];
    return nullptr;
}

void Camera::setKeyFrameInterpolator(unsigned int i,
                                     KeyFrameInterpolator *const kfi)
{
    if (kfi) {
        kfi_[i] = kfi;
    }
    else {
        kfi_.remove(i);
    }
}

void Camera::addKeyFrameToPath(unsigned int i)
{
    if (!kfi_.contains(i)) {
        setKeyFrameInterpolator(i, new KeyFrameInterpolator(frame()));
    }

    kfi_[i]->addKeyFrame(*(frame()));
}

void Camera::playPath(unsigned int i)
{
    if (kfi_.contains(i)) {
        if (kfi_[i]->interpolationIsStarted()) {
            kfi_[i]->stopInterpolation();
        }
        else {
            kfi_[i]->startInterpolation();
        }
    }
}

void Camera::resetPath(unsigned int i)
{
    if (kfi_.contains(i)) {
        if ((kfi_[i]->interpolationIsStarted())) {
            kfi_[i]->stopInterpolation();
        }
        else {
            kfi_[i]->resetInterpolation();
            kfi_[i]->interpolateAtTime(kfi_[i]->interpolationTime());
        }
    }
}

void Camera::deletePath(unsigned int i)
{
    if (kfi_.contains(i)) {
        kfi_[i]->stopInterpolation();
        delete kfi_[i];
        kfi_.remove(i);
    }
}

void Camera::drawAllPaths()
{
    for (auto it = kfi_.begin(), end = kfi_.end(); it != end; ++it) {
        (it.value())->drawPath(3, 5, sceneRadius());
    }
}

namespace {
// TODO: Use QMetaEnum
constexpr char kPerspectiveName[]{"PERSPECTIVE"};
constexpr char kOrthographicName[]{"ORTHOGRAPHIC"};
} // namespace

namespace key {
constexpr char kParameters[]{"Parameters"};
constexpr char kFov[]{"fieldOfView"};
constexpr char kZNear[]{"zNearCoefficient"};
constexpr char kZClipping[]{"zClippingCoefficient"};
constexpr char kOrthoCoeff[]{"orthoCoef"};
constexpr char kSceneRadius[]{"sceneRadius"};
constexpr char kSceneCenter[]{"SceneCenter"};
constexpr char kType[]{"type"};
constexpr char kStereo[]{"Stereo"};
constexpr char kIoDistance[]{"IODist"};
constexpr char kFocusDistance[]{"focusDistance"};
constexpr char kPhysicalScreenWidth[]{"physScreenWidth"};
constexpr char kManipulatedCameraFrame[]{"ManipulatedCameraFrame"};
constexpr char kKeyFrameInterpolator[]{"KeyFrameInterpolator"};
constexpr char kIndex[]{"index"};
} // namespace key

QDomElement Camera::domElement(const QString &name,
                               QDomDocument &document) const
{
    auto root = document.createElement(name);

    // Parameters
    auto paramNode = document.createElement(key::kParameters);
    paramNode.setAttribute(key::kFov, QString::number(fieldOfView()));
    paramNode.setAttribute(key::kZNear, QString::number(zNearCoefficient()));
    paramNode.setAttribute(key::kZClipping,
                           QString::number(zClippingCoefficient()));
    paramNode.setAttribute(key::kOrthoCoeff, QString::number(orthoCoef_));
    paramNode.setAttribute(key::kSceneRadius, QString::number(sceneRadius()));
    paramNode.appendChild(
        sceneCenter().domElement(key::kSceneCenter, document));

    switch (type()) {
        case Type::Perspective:
            paramNode.setAttribute(key::kType, kPerspectiveName);
            break;
        case Type::Orthographic:
            paramNode.setAttribute(key::kType, kOrthographicName);
            break;
        default:
            break;
    }
    root.appendChild(paramNode);

    // Stereo
    auto stereoNode = document.createElement(key::kStereo);
    stereoNode.setAttribute(key::kIoDistance, QString::number(IODistance()));
    stereoNode.setAttribute(key::kFocusDistance,
                            QString::number(focusDistance()));
    stereoNode.setAttribute(key::kPhysicalScreenWidth,
                            QString::number(physicalScreenWidth()));
    root.appendChild(stereoNode);

    root.appendChild(
        frame()->domElement(key::kManipulatedCameraFrame, document));

    // KeyFrame paths
    for (auto it = kfi_.begin(), end = kfi_.end(); it != end; ++it) {
        auto kfiNode =
            (it.value())->domElement(key::kKeyFrameInterpolator, document);
        kfiNode.setAttribute(key::kIndex, QString::number(it.key()));
        root.appendChild(kfiNode);
    }

    return root;
}

void Camera::initFromDOMElement(const QDomElement &element)
{
    QMutableMapIterator<unsigned int, KeyFrameInterpolator *> it(kfi_);
    while (it.hasNext()) {
        it.next();
        deletePath(it.key());
    }

    auto child = element.firstChild().toElement();
    while (!child.isNull()) {
        // Parameters
        if (child.tagName() == key::kParameters) {
            setFieldOfView(io::qrealFromDom(child, key::kFov, M_PI / 4.0));
            setZNearCoefficient(io::qrealFromDom(child, key::kZNear, 0.005));
            setZClippingCoefficient(
                io::qrealFromDom(child, key::kZClipping, sqrt(3.0)));
            orthoCoef_ = io::qrealFromDom(child, key::kOrthoCoeff,
                                          tan(fieldOfView() / 2.0));
            setSceneRadius(
                io::qrealFromDom(child, key::kSceneRadius, sceneRadius()));

            setType(Type::Perspective);
            const auto typeName = child.attribute(key::kType, kPerspectiveName);
            if (typeName == kPerspectiveName) {
                setType(Type::Perspective);
            }
            if (typeName == kOrthographicName) {
                setType(Type::Orthographic);
            }

            auto child2 = child.firstChild().toElement();
            while (!child2.isNull()) {
                // Although the scene does not change when a camera is loaded,
                // restore the saved center and radius values. Mainly useful
                // when a the viewer is restored on startup, with possible
                // additional cameras.
                if (child2.tagName() == key::kSceneCenter) {
                    setSceneCenter(Vec(child2));
                }

                child2 = child2.nextSibling().toElement();
            }
        }

        if (child.tagName() == key::kManipulatedCameraFrame) {
            frame()->initFromDOMElement(child);
        }

        if (child.tagName() == key::kStereo) {
            setIODistance(
                io::qrealFromDom(child, key::kIoDistance, kDefaultIoDistance));
            setFocusDistance(
                io::qrealFromDom(child, key::kFocusDistance, focusDistance()));
            setPhysicalScreenWidth(io::qrealFromDom(
                child, key::kPhysicalScreenWidth, kDefaultPhysicalScreenWidth));
        }

        if (child.tagName() == key::kKeyFrameInterpolator) {
            const auto index = io::uintFromDom(child, key::kIndex, 0u);
            setKeyFrameInterpolator(index, new KeyFrameInterpolator(frame()));
            if (auto kfi = keyFrameInterpolator(index)) {
                kfi->initFromDOMElement(child);
            }
        }

        child = child.nextSibling().toElement();
    }
}

void Camera::convertClickToLine(const QPoint &pixel, Vec &orig, Vec &dir) const
{
    switch (type()) {
        case Type::Perspective: {
            orig = position();
            dir = {((2. * pixel.x() / screenWidth()) - 1.) *
                       tan(fieldOfView() / 2.) * aspectRatio(),
                   ((2. * (screenHeight() - pixel.y()) / screenHeight()) - 1.) *
                       tan(fieldOfView() / 2.),
                   -1.};
            dir = worldCoordinatesOf(dir) - orig;
            dir.normalize();
        } break;
        case Type::Orthographic: {
            GLdouble w, h;
            getOrthoWidthHeight(w, h);
            orig = {(2. * pixel.x() / screenWidth() - 1.) * w,
                    -(2. * pixel.y() / screenHeight() - 1.) * h, 0.};
            orig = worldCoordinatesOf(orig);
            dir = viewDirection();
        } break;
        default:
            break;
    }
}

void Camera::draw(bool drawFarPlane, qreal scale) const
{
    glPushMatrix();
    glMultMatrixd(frame()->worldMatrix());

    // 0 is the upper left coordinates of the near corner, 1 for the far one
    Vec points[2];

    points[0].z = scale * zNear();
    points[1].z = scale * zFar();

    switch (type()) {
        case Type::Perspective: {
            points[0].y = points[0].z * tan(fieldOfView() / 2.0);
            points[0].x = points[0].y * aspectRatio();

            const qreal ratio = points[1].z / points[0].z;

            points[1].y = ratio * points[0].y;
            points[1].x = ratio * points[0].x;
        } break;
        case Type::Orthographic: {
            GLdouble hw, hh;
            getOrthoWidthHeight(hw, hh);
            points[0].x = points[1].x = scale * qreal(hw);
            points[0].y = points[1].y = scale * qreal(hh);
        } break;
        default:
            break;
    }

    const int farIndex = drawFarPlane ? 1 : 0;

    // Near and (optionally) far plane(s)
    glBegin(GL_QUADS);
    for (int i = farIndex; i >= 0; --i) {
        glNormal3d(0., 0., (i == 0) ? 1. : -1.);
        glVertex3d(points[i].x, points[i].y, -points[i].z);
        glVertex3d(-points[i].x, points[i].y, -points[i].z);
        glVertex3d(-points[i].x, -points[i].y, -points[i].z);
        glVertex3d(points[i].x, -points[i].y, -points[i].z);
    }
    glEnd();

    // Up arrow
    const qreal arrowHeight = 1.5 * points[0].y;
    const qreal baseHeight = 1.2 * points[0].y;
    const qreal arrowHalfWidth = 0.5 * points[0].x;
    const qreal baseHalfWidth = 0.3 * points[0].x;

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    // Base
    glBegin(GL_QUADS);
    glVertex3d(-baseHalfWidth, points[0].y, -points[0].z);
    glVertex3d(baseHalfWidth, points[0].y, -points[0].z);
    glVertex3d(baseHalfWidth, baseHeight, -points[0].z);
    glVertex3d(-baseHalfWidth, baseHeight, -points[0].z);
    glEnd();

    // Arrow
    glBegin(GL_TRIANGLES);
    glVertex3d(0.0, arrowHeight, -points[0].z);
    glVertex3d(-arrowHalfWidth, baseHeight, -points[0].z);
    glVertex3d(arrowHalfWidth, baseHeight, -points[0].z);
    glEnd();

    // Frustum lines
    switch (type()) {
        case Type::Perspective:
            glBegin(GL_LINES);
            glVertex3d(0.0, 0.0, 0.0);
            glVertex3d(points[farIndex].x, points[farIndex].y,
                       -points[farIndex].z);
            glVertex3d(0.0, 0.0, 0.0);
            glVertex3d(-points[farIndex].x, points[farIndex].y,
                       -points[farIndex].z);
            glVertex3d(0.0, 0.0, 0.0);
            glVertex3d(-points[farIndex].x, -points[farIndex].y,
                       -points[farIndex].z);
            glVertex3d(0.0, 0.0, 0.0);
            glVertex3d(points[farIndex].x, -points[farIndex].y,
                       -points[farIndex].z);
            glEnd();
            break;
        case Type::Orthographic:
            if (drawFarPlane) {
                glBegin(GL_LINES);
                glVertex3d(points[0].x, points[0].y, -points[0].z);
                glVertex3d(points[1].x, points[1].y, -points[1].z);
                glVertex3d(-points[0].x, points[0].y, -points[0].z);
                glVertex3d(-points[1].x, points[1].y, -points[1].z);
                glVertex3d(-points[0].x, -points[0].y, -points[0].z);
                glVertex3d(-points[1].x, -points[1].y, -points[1].z);
                glVertex3d(points[0].x, -points[0].y, -points[0].z);
                glVertex3d(points[1].x, -points[1].y, -points[1].z);
                glEnd();
            }
            break;
        default:
            break;
    }

    glPopMatrix();
}

void Camera::getFrustumPlanesCoefficients(GLdouble coef[6][4]) const
{
    // Computed once and for all
    const Vec pos = position();
    const Vec viewDir = viewDirection();
    const Vec up = upVector();
    const Vec right = rightVector();
    const qreal posViewDir = pos * viewDir;

    static Vec normal[6];
    static GLdouble dist[6];

    switch (type()) {
        case Type::Perspective: {
            const qreal hhfov = horizontalFieldOfView() / 2.0;
            const qreal chhfov = cos(hhfov);
            const qreal shhfov = sin(hhfov);
            normal[0] = -shhfov * viewDir;
            normal[1] = normal[0] + chhfov * right;
            normal[0] = normal[0] - chhfov * right;

            normal[2] = -viewDir;
            normal[3] = viewDir;

            const qreal hfov = fieldOfView() / 2.0;
            const qreal chfov = cos(hfov);
            const qreal shfov = sin(hfov);
            normal[4] = -shfov * viewDir;
            normal[5] = normal[4] - chfov * up;
            normal[4] = normal[4] + chfov * up;

            for (int i = 0; i < 2; ++i) dist[i] = pos * normal[i];
            for (int j = 4; j < 6; ++j) dist[j] = pos * normal[j];

            // Natural equations are:
            // dist[0,1,4,5] = pos * normal[0,1,4,5];
            // dist[2] = (pos + zNear() * viewDir) * normal[2];
            // dist[3] = (pos + zFar()  * viewDir) * normal[3];

            // 2 times less computations using expanded/merged equations. Dir
            // vectors are normalized.
            const qreal posRightCosHH = chhfov * pos * right;
            dist[0] = -shhfov * posViewDir;
            dist[1] = dist[0] + posRightCosHH;
            dist[0] = dist[0] - posRightCosHH;
            const qreal posUpCosH = chfov * pos * up;
            dist[4] = -shfov * posViewDir;
            dist[5] = dist[4] - posUpCosH;
            dist[4] = dist[4] + posUpCosH;

            break;
        }
        case Type::Orthographic:
            normal[0] = -right;
            normal[1] = right;
            normal[4] = up;
            normal[5] = -up;

            GLdouble hw, hh;
            getOrthoWidthHeight(hw, hh);
            dist[0] = (pos - hw * right) * normal[0];
            dist[1] = (pos + hw * right) * normal[1];
            dist[4] = (pos + hh * up) * normal[4];
            dist[5] = (pos - hh * up) * normal[5];
            break;
        default:
            break;
    }

    // Front and far planes are identical for both camera types.
    normal[2] = -viewDir;
    normal[3] = viewDir;
    dist[2] = -posViewDir - zNear();
    dist[3] = posViewDir + zFar();

    for (int i = 0; i < 6; ++i) {
        coef[i][0] = GLdouble(normal[i].x);
        coef[i][1] = GLdouble(normal[i].y);
        coef[i][2] = GLdouble(normal[i].z);
        coef[i][3] = dist[i];
    }
}

void Camera::onFrameModified()
{
    projectionMatrixIsUpToDate_ = false;
    modelViewMatrixIsUpToDate_ = false;
}

void Camera::setHorizontalFieldOfView(qreal hfov)
{
    setFieldOfView(2.0 * atan(tan(hfov / 2.0) / aspectRatio()));
}

void Camera::setAspectRatio(qreal aspect)
{
    setScreenWidthAndHeight(int(100.0 * aspect), 100);
}

void Camera::setZNearCoefficient(qreal coef)
{
    zNearCoef_ = coef;
    projectionMatrixIsUpToDate_ = false;
}

void Camera::setZClippingCoefficient(qreal coef)
{
    zClippingCoef_ = coef;
    projectionMatrixIsUpToDate_ = false;
}

qreal Camera::IODistance() const { return IODistance_; }

void Camera::setIODistance(qreal distance) { IODistance_ = distance; }

qreal Camera::physicalDistanceToScreen() const
{
    return physicalScreenWidth() / 2.0 / tan(horizontalFieldOfView() / 2.0);
}

qreal Camera::physicalScreenWidth() const { return physicalScreenWidth_; }

void Camera::setPhysicalScreenWidth(qreal width)
{
    physicalScreenWidth_ = width;
}

qreal Camera::focusDistance() const { return focusDistance_; }

void Camera::setFocusDistance(qreal distance) { focusDistance_ = distance; }
