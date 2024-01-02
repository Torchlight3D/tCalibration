#pragma once

#include <QObject>
#include <QMap>

#include "quaternion.h"
#include "vec.h"

class QtOpenGLViewer;

namespace viewer {

class Frame;
class KeyFrameInterpolator;
class ManipulatedCameraFrame;

class Camera : public QObject
{
    friend class ::QtOpenGLViewer;

    Q_OBJECT

public:
    Camera();
    Camera(const Camera &camera);
    Camera &operator=(const Camera &camera);
    virtual ~Camera();

    /// Properties
    enum class Type
    {
        Perspective,
        Orthographic
    };
    Type type() const;
    Q_SLOT void setType(Type type);

    Vec position() const;
    Q_SLOT void setPosition(const Vec &pos);

    Quaternion orientation() const;
    Q_SLOT void setOrientation(const Quaternion &q);
    Q_SLOT void setOrientation(qreal theta, qreal phi);

    Vec upVector() const;
    Q_SLOT void setUpVector(const Vec &up, bool noMove = true);

    Vec viewDirection() const;
    Q_SLOT void setViewDirection(const Vec &direction);

    Vec rightVector() const;

    void setFromModelViewMatrix(const GLdouble *const modelViewMatrix);
    void setFromProjectionMatrix(const qreal matrix[12]);

    qreal fieldOfView() const;
    inline auto verticalFieldOfView() const { return fieldOfView(); }
    Q_SLOT void setFieldOfView(qreal fov);

    qreal horizontalFieldOfView() const;
    Q_SLOT void setHorizontalFieldOfView(qreal hfov);

    qreal aspectRatio() const;
    Q_SLOT void setAspectRatio(qreal aspect);

    int screenWidth() const;
    int screenHeight() const;
    Q_SLOT void setScreenWidthAndHeight(int width, int height);

    qreal devicePixelRatio() const;
    Q_SLOT void setDevicePixelRatio(qreal ratio);

    void getViewport(GLint viewport[4]) const;
    qreal pixelGLRatio(const Vec &position) const;

    qreal zNearCoefficient() const;
    Q_SLOT void setZNearCoefficient(qreal coefficient);

    qreal zClippingCoefficient() const;
    Q_SLOT void setZClippingCoefficient(qreal coefficient);

    qreal sceneRadius() const;
    Q_SLOT void setSceneRadius(qreal radius);

    Vec sceneCenter() const;
    Q_SLOT void setSceneCenter(const Vec &center);

    qreal distanceToSceneCenter() const;

    qreal flySpeed() const;
    Q_SLOT void setFlySpeed(qreal speed);

    qreal IODistance() const;
    Q_SLOT void setIODistance(qreal distance);

    qreal physicalDistanceToScreen() const;

    qreal physicalScreenWidth() const;
    Q_SLOT void setPhysicalScreenWidth(qreal width);

    qreal focusDistance() const;
    Q_SLOT void setFocusDistance(qreal distance);

    Vec pivotPoint() const;
    Q_SLOT void setPivotPoint(const Vec &point);

    ManipulatedCameraFrame *frame() const;
    Q_SLOT void setFrame(ManipulatedCameraFrame *const mcf);

    KeyFrameInterpolator *keyFrameInterpolator(unsigned int i) const;
    Q_SLOT void setKeyFrameInterpolator(unsigned int i,
                                        KeyFrameInterpolator *const kfi);

    virtual qreal zNear() const;
    virtual qreal zFar() const;
    virtual void getOrthoWidthHeight(GLdouble &halfWidth,
                                     GLdouble &halfHeight) const;
    void getFrustumPlanesCoefficients(GLdouble coef[6][4]) const;

    virtual void loadProjectionMatrix(bool reset = true) const;
    virtual void loadModelViewMatrix(bool reset = true) const;
    void computeProjectionMatrix() const;
    void computeModelViewMatrix() const;

    virtual void loadProjectionMatrixStereo(bool leftBuffer = true) const;
    virtual void loadModelViewMatrixStereo(bool leftBuffer = true) const;

    void getProjectionMatrix(GLfloat m[16]) const;
    void getProjectionMatrix(GLdouble m[16]) const;

    void getModelViewMatrix(GLfloat m[16]) const;
    void getModelViewMatrix(GLdouble m[16]) const;

    void getModelViewProjectionMatrix(GLfloat m[16]) const;
    void getModelViewProjectionMatrix(GLdouble m[16]) const;

    virtual void draw(bool drawFarPlane = true, qreal scale = 1.0) const;

    Vec cameraCoordinatesOf(const Vec &src) const;
    Vec worldCoordinatesOf(const Vec &src) const;
    void getCameraCoordinatesOf(const qreal src[3], qreal res[3]) const;
    void getWorldCoordinatesOf(const qreal src[3], qreal res[3]) const;

    Vec projectedCoordinatesOf(const Vec &src,
                               const Frame *frame = nullptr) const;
    Vec unprojectedCoordinatesOf(const Vec &src,
                                 const Frame *frame = nullptr) const;
    void getProjectedCoordinatesOf(const qreal src[3], qreal res[3],
                                   const Frame *frame = nullptr) const;
    void getUnprojectedCoordinatesOf(const qreal src[3], qreal res[3],
                                     const Frame *frame = nullptr) const;
    void convertClickToLine(const QPoint &pixel, Vec &orig, Vec &dir) const;
    Vec pointUnderPixel(const QPoint &pixel, bool &found) const;

    virtual QDomElement domElement(const QString &name,
                                   QDomDocument &document) const;
    Q_SLOT virtual void initFromDOMElement(const QDomElement &element);

public slots:

    void lookAt(const Vec &target);
    void showEntireScene();
    void fitSphere(const Vec &center, qreal radius);
    void fitBoundingBox(const Vec &min, const Vec &max);
    void fitScreenRegion(const QRect &rectangle);
    void centerScene();
    void interpolateToZoomOnPixel(const QPoint &pixel);
    void interpolateToFitScene();
    void interpolateTo(const Frame &fr, qreal duration);

    void setFOVToFitScene();

    bool setSceneCenterFromPixel(const QPoint &pixel);
    void setSceneBoundingBox(const Vec &min, const Vec &max);

    bool setPivotPointFromPixel(const QPoint &pixel);

    virtual void addKeyFrameToPath(unsigned int i);
    virtual void playPath(unsigned int i);
    virtual void deletePath(unsigned int i);
    virtual void resetPath(unsigned int i);
    virtual void drawAllPaths();

private slots:
    void onFrameModified();

private:
    ManipulatedCameraFrame *frame_;

    int screenWidth_, screenHeight_; // in pixels
    qreal fieldOfView_;              // in radians
    Vec sceneCenter_;
    qreal sceneRadius_; // OpenGL units
    qreal zNearCoef_;
    qreal zClippingCoef_;
    qreal orthoCoef_;
    qreal devicePixelRatio_;
    Type type_;
    mutable GLdouble modelViewMatrix_[16]; // Buffered model view matrix.
    mutable bool modelViewMatrixIsUpToDate_;
    mutable GLdouble projectionMatrix_[16]; // Buffered projection matrix.
    mutable bool projectionMatrixIsUpToDate_;

    qreal IODistance_;          // inter-ocular distance, in meters
    qreal focusDistance_;       // in scene units
    qreal physicalScreenWidth_; // in meters

    QMap<unsigned int, KeyFrameInterpolator *> kfi_;
    KeyFrameInterpolator *interpolationKfi_;
};

} // namespace viewer
