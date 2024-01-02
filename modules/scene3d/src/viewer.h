#pragma once

#include <QElapsedTimer>
#include <QMap>
#include <QOpenGLWidget>
#include <QTimer>

#include "vec.h"

class QTabWidget;

namespace viewer {
class Camera;
class MouseGrabber;
class ManipulatedFrame;
class ManipulatedCameraFrame;
} // namespace viewer

class QtOpenGLViewerPrivate;
class QtOpenGLViewer : public QOpenGLWidget
{
    Q_OBJECT

public:
    explicit QtOpenGLViewer(QWidget *parent = nullptr,
                            Qt::WindowFlags flags = Qt::WindowFlags());
    virtual ~QtOpenGLViewer();

    /// Properties
    bool axisIsDrawn() const;
    Q_SLOT void setAxisIsDrawn(bool draw = true);
    Q_SLOT inline void toggleAxisIsDrawn() { setAxisIsDrawn(!axisIsDrawn()); }
    Q_SIGNAL void axisIsDrawnChanged(bool drawn);

    bool gridIsDrawn() const;
    Q_SLOT void setGridIsDrawn(bool draw = true);
    Q_SLOT inline void toggleGridIsDrawn() { setGridIsDrawn(!gridIsDrawn()); }
    Q_SIGNAL void gridIsDrawnChanged(bool drawn);

    bool FPSIsDisplayed() const;
    Q_SLOT void setFPSIsDisplayed(bool display = true);
    Q_SLOT inline void toggleFPSIsDisplayed()
    {
        setFPSIsDisplayed(!FPSIsDisplayed());
    }
    Q_SIGNAL void FPSIsDisplayedChanged(bool displayed);

    bool textIsEnabled() const;
    Q_SLOT void setTextIsEnabled(bool enable = true);
    Q_SLOT inline void toggleTextIsEnabled()
    {
        setTextIsEnabled(!textIsEnabled());
    }
    Q_SIGNAL void textIsEnabledChanged(bool enabled);

    bool cameraIsEdited() const;
    Q_SLOT void setCameraIsEdited(bool edit = true);
    Q_SLOT inline void toggleCameraIsEdited()
    {
        setCameraIsEdited(!cameraIsEdited());
    }
    Q_SIGNAL void cameraIsEditedChanged(bool edited);

    QColor backgroundColor() const;
    Q_SLOT void setBackgroundColor(const QColor &color);

    QColor foregroundColor() const;
    Q_SLOT void setForegroundColor(const QColor &color);

    qreal sceneRadius() const;
    Q_SLOT virtual void setSceneRadius(qreal radius);

    viewer::Vec sceneCenter() const;
    Q_SLOT virtual void setSceneCenter(const viewer::Vec &center);

    viewer::Camera *camera() const;
    Q_SLOT void setCamera(viewer::Camera *const camera);

    viewer::ManipulatedFrame *manipulatedFrame() const;
    Q_SLOT void setManipulatedFrame(viewer::ManipulatedFrame *frame);

public slots:
    void setSceneBoundingBox(const viewer::Vec &min, const viewer::Vec &max);
    void showEntireScene();

public:
    viewer::MouseGrabber *mouseGrabber() const;
    Q_SLOT void setMouseGrabber(viewer::MouseGrabber *mouseGrabber);
    Q_SIGNAL void mouseGrabberChanged(viewer::MouseGrabber *mouseGrabber);

    bool mouseGrabberIsEnabled(const viewer::MouseGrabber *const mouseGrabber);
    Q_SLOT void setMouseGrabberIsEnabled(
        const viewer::MouseGrabber *const mouseGrabber, bool enabled = true);

public:
    qreal aspectRatio() const;

    qreal currentFPS() const;

    bool isFullScreen() const;
    Q_SLOT void setFullScreen(bool fullScreen = true);
    Q_SLOT inline void toggleFullScreen() { setFullScreen(!isFullScreen()); }

    bool displaysInStereo() const;
    Q_SLOT void setStereoDisplay(bool stereo = true);
    Q_SLOT inline void toggleStereoDisplay()
    {
        setStereoDisplay(!displaysInStereo());
    }
    Q_SIGNAL void stereoChanged(bool on);

    QSize sizeHint() const override { return {600, 400}; }

public slots:
    void toggleCameraMode();

private:
    bool cameraIsInRotateMode() const;

public:
    static void drawArrow(qreal length = 1.0, qreal radius = -1.0,
                          int subdivisions = 12);
    static void drawArrow(const viewer::Vec &from, const viewer::Vec &to,
                          qreal radius = -1.0, int subdivisions = 12);
    static void drawAxis(qreal length = 1.0);
    static void drawGrid(qreal size = 1.0, int subdivisions = 10);

    virtual void startScreenCoordinatesSystem(bool upward = false) const;
    virtual void stopScreenCoordinatesSystem() const;

    void drawText(int x, int y, const QString &text, const QFont &font = {});
    void displayMessage(const QString &message, int delay = 2000);
    // void draw3DText(const viewer::Vec& pos, const viewer::Vec& normal,
    // const QString& string, GLfloat height=0.1);

protected:
    virtual void drawLight(GLenum light, qreal scale = 1.0) const;

private:
    void displayFPS();
    void drawVectorial() { paintGL(); }

    friend void drawVectorial(void *param);

#ifdef DOXYGEN
public:
    int width() const;
    int height() const;

    virtual void update();

    bool isValid() const;

    virtual void makeCurrent();

    bool hasMouseTracking() const;

public slots:
    virtual void resize(int width, int height);
    virtual void setMouseTracking(bool enable);
#endif

public:
    /// Snapshot
    const QString &snapshotFileName() const;
    Q_SLOT void setSnapshotFileName(const QString &name);

    const QString &snapshotFormat() const;
    Q_SLOT void setSnapshotFormat(const QString &format);

    int snapshotCounter() const;
    Q_SLOT void setSnapshotCounter(int counter);

    int snapshotQuality();
    Q_SLOT void setSnapshotQuality(int quality);

public slots:
    void saveSnapshot(bool automatic = true, bool overwrite = false);
    void saveSnapshot(const QString &fileName, bool overwrite = false);
    bool openSnapshotFormatDialog();
    void snapshotToClipboard();

private:
    bool saveImageSnapshot(const QString &fileName);

public:
    QFont scaledFont(const QFont &font) const;

public:
    GLuint bufferTextureId() const;

    qreal bufferTextureMaxU() const;
    qreal bufferTextureMaxV() const;

    void renderText(int x, int y, const QString &str, const QFont &font = {});
    void renderText(double x, double y, double z, const QString &str,
                    const QFont &font = {});

public slots:
    void copyBufferToTexture(GLint internalFormat, GLenum format = GL_NONE);

public:
    int animationPeriod() const;
    Q_SLOT void setAnimationPeriod(int period);

    bool animationIsStarted() const;
    Q_SLOT virtual void startAnimation();
    Q_SLOT virtual void stopAnimation();
    Q_SLOT inline void toggleAnimation()
    {
        if (animationIsStarted())
            stopAnimation();
        else
            startAnimation();
    }

public slots:
    virtual void animate() { emit animateNeeded(); }

signals:
    void viewerInitialized();
    void drawNeeded();
    void drawFinished(bool automatic);
    void animateNeeded();
    void helpRequired();
    void pointSelected(const QMouseEvent *e);

public:
    virtual QString helpString() const { return tr("No help available."); }

    virtual QString mouseString() const;
    virtual QString keyboardString() const;

public slots:
    virtual void help();

protected:
    QTabWidget *helpWidget();

protected:
    void resizeGL(int width, int height) override;
    void initializeGL() override;

    virtual void init() { emit viewerInitialized(); }

    void paintGL() override;
    virtual void preDraw();
    virtual void preDrawStereo(bool leftBuffer = true);

    virtual void draw() {}
    virtual void fastDraw();
    virtual void postDraw();

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
    void timerEvent(QTimerEvent *event) override;
    void closeEvent(QCloseEvent *event) override;

public:
    // TODO: Change interface name, name -> id ?
    int selectedName() const;
    Q_SLOT void setSelectedName(int id);

    int selectBufferSize() const;
    Q_SLOT void setSelectBufferSize(int size);

    int selectRegionWidth() const;
    Q_SLOT void setSelectRegionWidth(int width);

    int selectRegionHeight() const;
    Q_SLOT void setSelectRegionHeight(int height);

    GLuint *selectBuffer();

public slots:
    virtual void select(const QMouseEvent *event);
    virtual void select(const QPoint &point);

protected:
    virtual void beginSelection(const QPoint &point);
    virtual void endSelection(const QPoint &point);
    virtual void postSelection(const QPoint &point) { Q_UNUSED(point); }

    virtual void drawWithNames() {}

public:
    enum KeyboardAction
    {
        ShowWorldFrame,
        ShowWorldGrid,
        ShowFPS,
        EnableText,
        ExitViewer,
        SaveScreenshot,
        CameraMode,
        FullScreen,
        ToggleStereoMode,
        ToggleAnimation,
        Help,
        EditCamera,
        MoveCameraLeft,
        MoveCameraRight,
        MoveCameraUp,
        MoveCameraDown,
        IncreaseFlySpeed,
        DecreaseFlySpeed,
        SnapshotToClipboard
    };

    /// Key shortcuts
    int shortcut(KeyboardAction action) const;
    Q_SLOT void setShortcut(KeyboardAction action, const QKeyCombination &comb);
    Q_SLOT void clearShortcuts();

    Qt::Key pathKey(unsigned int index) const;
    Q_SLOT virtual void setPathKey(int key, unsigned int index = 0);

    Qt::KeyboardModifiers playPathKeyboardModifiers() const;
    Q_SLOT virtual void setPlayPathKeyboardModifiers(
        Qt::KeyboardModifiers modifiers);

    Qt::KeyboardModifiers addKeyFrameKeyboardModifiers() const;
    Q_SLOT virtual void setAddKeyFrameKeyboardModifiers(
        Qt::KeyboardModifiers modifiers);

public slots:
    // User customize description
    void setKeyDescription(int key, const QString &description);

public:
    enum MouseHandler
    {
        CAMERA,
        FRAME
    };

    enum ClickAction
    {
        NoClick,
        ZoomOnPixel,
        ZoomToFit,
        Select,
        RAP_FROM_PIXEL,
        RAP_IS_CENTER,
        CenterFrame,
        CenterScene,
        ShowEntireScene,
        AlignFrame,
        AlignCamera
    };

    enum MouseAction
    {
        NoMouseAction,
        Rotate,
        Zoom,
        Translate,
        MoveForward,
        LookForward,
        MoveBackward,
        ScreenRotate,
        Roll,
        Drive,
        ScreenTranslate,
        ZoomOnRegion
    };

    MouseAction mouseAction(Qt::Key key, Qt::KeyboardModifiers modifiers,
                            Qt::MouseButton button) const;
    int mouseHandler(Qt::Key key, Qt::KeyboardModifiers modifiers,
                     Qt::MouseButton button) const;
    void getMouseActionBinding(MouseHandler handler, MouseAction action,
                               bool withConstraint, Qt::Key &key,
                               Qt::KeyboardModifiers &modifiers,
                               Qt::MouseButton &button) const;

    ClickAction clickAction(
        Qt::Key key, Qt::KeyboardModifiers modifiers, Qt::MouseButton button,
        bool doubleClick = false,
        Qt::MouseButtons buttonsBefore = Qt::NoButton) const;
    void getClickActionBinding(ClickAction action, Qt::Key &key,
                               Qt::KeyboardModifiers &modifiers,
                               Qt::MouseButton &button, bool &doubleClick,
                               Qt::MouseButtons &buttonsBefore) const;

    MouseAction wheelAction(Qt::Key key, Qt::KeyboardModifiers modifiers) const;
    int wheelHandler(Qt::Key key, Qt::KeyboardModifiers modifiers) const;
    void getWheelActionBinding(MouseHandler handler, MouseAction action,
                               bool withConstraint, Qt::Key &key,
                               Qt::KeyboardModifiers &modifiers) const;

public slots:
    // Click
    void setMouseBinding(Qt::Key key, Qt::KeyboardModifiers modifiers,
                         Qt::MouseButton button, MouseHandler handler,
                         MouseAction action, bool withConstraint = true);
    inline void setMouseBinding(Qt::KeyboardModifiers modifiers,
                                Qt::MouseButton button, MouseHandler handler,
                                MouseAction action, bool withConstraint = true)
    {
        setMouseBinding(Qt::Key(0), modifiers, button, handler, action,
                        withConstraint);
    }

    // DoubleClick
    void setMouseBinding(Qt::Key key, Qt::KeyboardModifiers modifiers,
                         Qt::MouseButton button, ClickAction action,
                         bool doubleClick = false,
                         Qt::MouseButtons buttonsBefore = Qt::NoButton);
    inline void setMouseBinding(Qt::KeyboardModifiers modifiers,
                                Qt::MouseButton button, ClickAction action,
                                bool doubleClick = false,
                                Qt::MouseButtons buttonsBefore = Qt::NoButton)
    {
        setMouseBinding(Qt::Key(0), modifiers, button, action, doubleClick,
                        buttonsBefore);
    }

    // Wheel
    void setWheelBinding(Qt::Key key, Qt::KeyboardModifiers modifiers,
                         MouseHandler handler, MouseAction action,
                         bool withConstraint = true);
    inline void setWheelBinding(Qt::KeyboardModifiers modifiers,
                                MouseHandler handler, MouseAction action,
                                bool withConstraint = true)
    {
        setWheelBinding(Qt::Key(0), modifiers, handler, action, withConstraint);
    }

    // Custom mouse binding type
    void setMouseBindingDescription(
        Qt::Key key, Qt::KeyboardModifiers modifiers, Qt::MouseButton button,
        QString description, bool doubleClick = false,
        Qt::MouseButtons buttonsBefore = Qt::NoButton);
    inline void setMouseBindingDescription(
        Qt::KeyboardModifiers modifiers, Qt::MouseButton button,
        QString description, bool doubleClick = false,
        Qt::MouseButtons buttonsBefore = Qt::NoButton)
    {
        setMouseBindingDescription(Qt::Key(0), modifiers, button, description,
                                   doubleClick, buttonsBefore);
    }

    void clearMouseBindings();

public:
    QString stateFileName() const;
    Q_SLOT void setStateFileName(const QString &name);

    virtual QDomElement domElement(const QString &name,
                                   QDomDocument &document) const;
    Q_SLOT virtual void initFromDOMElement(const QDomElement &element);

public slots:
    virtual void saveStateToFile(); // cannot be const because of QMessageBox
    virtual bool restoreStateFromFile();

public:
    static const QList<QtOpenGLViewer *> &QGLViewerPool();
    static int QGLViewerIndex(const QtOpenGLViewer *const viewer);

public:
    virtual void setVisualHintsMask(int mask, int delay = 2000);
    virtual void drawVisualHints();

public slots:
    virtual void resetVisualHints();

private slots:
    // Patch for a Qt bug with fullScreen on startup
    void delayedFullScreen()
    {
        move(prevPos_);
        setFullScreen();
    }
    void hideMessage();

private:
    void defaultConstructor();
    void handleKeyboardAction(KeyboardAction id);

    void setDefaultShortcuts();
    QString cameraPathKeysString() const;

    void setDefaultMouseBindings();
    void performClickAction(ClickAction ca, const QMouseEvent *const e);

    void initializeSnapshotFormats();
    QImage frameBufferSnapshot();

    void connectAllCameraKFIInterpolatedSignals(bool connection = true);

private:
    Q_DISABLE_COPY(QtOpenGLViewer)

    viewer::Camera *camera_;
    bool cameraIsEdited_;
    qreal previousCameraZClippingCoefficient_;
    unsigned int previousPathId_; // double key press recognition
    QColor backgroundColor_, foregroundColor_;

    bool axisIsDrawn_;    // world axis
    bool gridIsDrawn_;    // world XY grid
    bool FPSIsDisplayed_; // Frame Per Seconds
    bool textIsEnabled_;  // drawText() actually draws text or not
    bool stereo_;         // stereo display
    bool fullScreen_;     // full screen mode
    QPoint prevPos_;      // Previous window position, used for full screen mode

    // Animation
    bool animationStarted_; // animation mode started
    int animationPeriod_;   // period in msecs
    int animationTimerId_;

    // FPS
    QElapsedTimer fpsTime_;
    unsigned int fpsCounter_;
    QString fpsString_;
    qreal fps_;

    QString message_;
    bool displayMessage_;
    QTimer messageTimer_;

    viewer::ManipulatedFrame *manipulatedFrame_;
    bool manipulatedFrameIsACamera_;

    viewer::MouseGrabber *mouseGrabber_;
    bool mouseGrabberIsAManipulatedFrame_;
    bool mouseGrabberIsAManipulatedCameraFrame_;
    QMap<size_t, bool> disabledMouseGrabbers_;

    int selectRegionWidth_, selectRegionHeight_;
    int selectBufferSize_;
    GLuint *selectBuffer_;
    int selectedObjectId_;

    int visualHint_;

    QMap<KeyboardAction, QString> keyboardActionDescription_;
    QMap<KeyboardAction, int> keyboardBinding_;
    QMap<int, QString> keyDescription_;

    QMap<Qt::Key, unsigned int> pathIndex_;
    Qt::KeyboardModifiers addKeyFrameKeyboardModifiers_,
        playPathKeyboardModifiers_;

    GLuint bufferTextureId_;
    qreal bufferTextureMaxU_, bufferTextureMaxV_;
    int bufferTextureWidth_, bufferTextureHeight_;
    unsigned int previousBufferTextureFormat_;
    int previousBufferTextureInternalFormat_;

private:
    struct ClickBindingPrivate;
    struct MouseActionPrivate;
    struct MouseBindingPrivate;
    struct WheelBindingPrivate;

    // This class is used internally for screenshot that require tiling (image
    // size size different from window size). Only in that case, is the private
    // tileRegion_ pointer non null. It then contains the current tiled region,
    // which is used by startScreenCoordinatesSystem to adapt the coordinate
    // system. Not using it would result in a tiled drawing of the parts that
    // use startScreenCoordinatesSystem. Also used by scaledFont for same
    // purposes.
    struct TileRegion
    {
        qreal xMin, yMin, xMax, yMax, textScale;
    };

    QMap<ClickBindingPrivate, QString> mouseDescription_;
    QMap<MouseBindingPrivate, MouseActionPrivate> mouseBinding_;
    QMap<WheelBindingPrivate, MouseActionPrivate> wheelBinding_;
    QMap<ClickBindingPrivate, ClickAction> clickBinding_;
    Qt::Key currentlyPressedKey_;
    QString snapshotFileName_, snapshotFormat_;
    int snapshotCounter_, snapshotQuality_;
    TileRegion *tileRegion_;
    static QList<QtOpenGLViewer *> QGLViewerPool_;
    QString stateFileName_;
    QTabWidget *helpWidget_;
};
