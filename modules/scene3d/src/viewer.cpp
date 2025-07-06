#include "viewer.h"

#ifdef _WIN32
#include <windows.h>
#include <GL/glu.h>
#elif
#include <GL/glu.h>
#endif

#include <QApplication>
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QImage>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QScreen>
#include <QTabWidget>
#include <QTextEdit>
#include <QTextStream>
#include <QTimer>
#include <QUrl>
#include <QtAlgorithms>

#include "camera.h"
#include "key_frame_interpolator.h"
#include "manipulated_camera_frame.h"
#include "dom_utils.h"

using namespace viewer;

namespace {

inline auto Tr(const char *s, const char *c = nullptr, int n = -1)
{
    return QtOpenGLViewer::tr(s, c, n);
}

inline QString QGLViewerVersionString() { return {"Dummy Version"}; }

QString mouseButtonsString(Qt::MouseButtons btns)
{
    const QString kAmpersand{" & "};

    QString result{};
    bool addAmpersand{false};
    if (btns.testFlag(Qt::LeftButton)) {
        result += Tr("Left", "left mouse button");
        addAmpersand = true;
    }
    if (btns.testFlag(Qt::MiddleButton)) {
        if (addAmpersand)
            result += kAmpersand;
        result += Tr("Middle", "middle mouse button");
        addAmpersand = true;
    }
    if (btns.testFlag(Qt::RightButton)) {
        if (addAmpersand)
            result += kAmpersand;
        result += Tr("Right", "right mouse button");
    }
    return result;
}

inline QString keyString(const QKeyCombination &comb)
{
    return QKeySequence{comb}.toString(QKeySequence::NativeText);
}

inline QString keyString(int comb)
{
    return keyString(QKeyCombination::fromCombined(comb));
}

QString tableLine(const QString &left, const QString &right)
{
    static bool even = false;
    const QString tdtd("</b></td><td>");
    const QString tdtr("</td></tr>\n");

    QString res("<tr bgcolor=\"");

    if (even)
        res += "#eeeeff\">";
    else
        res += "#ffffff\">";
    res += "<td><b>" + left + tdtd + right + tdtr;
    even = !even;

    return res;
}

QString mouseActionString(QtOpenGLViewer::MouseAction ma)
{
    switch (ma) {
        case QtOpenGLViewer::NoMouseAction:
            return {};
        case QtOpenGLViewer::Rotate:
            return Tr("Rotates", "ROTATE mouse action");
        case QtOpenGLViewer::Zoom:
            return Tr("Zooms", "ZOOM mouse action");
        case QtOpenGLViewer::Translate:
            return Tr("Translates", "TRANSLATE mouse action");
        case QtOpenGLViewer::MoveForward:
            return Tr("Moves forward", "MOVE_FORWARD mouse action");
        case QtOpenGLViewer::LookForward:
            return Tr("Looks around", "LOOK_AROUND mouse action");
        case QtOpenGLViewer::MoveBackward:
            return Tr("Moves backward", "MOVE_BACKWARD mouse action");
        case QtOpenGLViewer::ScreenRotate:
            return Tr("Rotates in screen plane", "SCREEN_ROTATE mouse action");
        case QtOpenGLViewer::Roll:
            return Tr("Rolls", "ROLL mouse action");
        case QtOpenGLViewer::Drive:
            return Tr("Drives", "DRIVE mouse action");
        case QtOpenGLViewer::ScreenTranslate:
            return Tr("Horizontally/Vertically translates",
                      "SCREEN_TRANSLATE mouse action");
        case QtOpenGLViewer::ZoomOnRegion:
            return Tr("Zooms on region for", "ZOOM_ON_REGION mouse action");
        default:
            break;
    }

    return {};
}

QString clickActionString(QtOpenGLViewer::ClickAction ca)
{
    switch (ca) {
        case QtOpenGLViewer::NoClick:
            return QString();
        case QtOpenGLViewer::ZoomOnPixel:
            return Tr("Zooms on pixel", "ZOOM_ON_PIXEL click action");
        case QtOpenGLViewer::ZoomToFit:
            return Tr("Zooms to fit scene", "ZOOM_TO_FIT click action");
        case QtOpenGLViewer::Select:
            return Tr("Selects", "SELECT click action");
        case QtOpenGLViewer::RAP_FROM_PIXEL:
            return Tr("Sets pivot point", "RAP_FROM_PIXEL click action");
        case QtOpenGLViewer::RAP_IS_CENTER:
            return Tr("Resets pivot point", "RAP_IS_CENTER click action");
        case QtOpenGLViewer::CenterFrame:
            return Tr("Centers manipulated frame", "CENTER_FRAME click action");
        case QtOpenGLViewer::CenterScene:
            return Tr("Centers scene", "CENTER_SCENE click action");
        case QtOpenGLViewer::ShowEntireScene:
            return Tr("Shows entire scene", "SHOW_ENTIRE_SCENE click action");
        case QtOpenGLViewer::AlignFrame:
            return Tr("Aligns manipulated frame", "ALIGN_FRAME click action");
        case QtOpenGLViewer::AlignCamera:
            return Tr("Aligns camera", "ALIGN_CAMERA click action");
        default:
            break;
    }

    return {};
}

inline void saveStateToFileForAllViewers()
{
    for (auto *viewer : QtOpenGLViewer::QGLViewerPool()) {
        if (viewer)
            viewer->saveStateToFile();
    }
}

inline bool isValidShortcutKey(int key)
{
    return (key >= Qt::Key_Any && key < Qt::Key_Escape) ||
           (key >= Qt::Key_F1 && key <= Qt::Key_F35);
}

} // namespace

///------- QGLViewer::MouseActionPrivate starts from here
struct QtOpenGLViewer::MouseActionPrivate
{
    MouseHandler handler;
    MouseAction action;
    bool withConstraint;

    bool operator==(const MouseActionPrivate &other) const
    {
        return handler == other.handler && action == other.action &&
               withConstraint == other.withConstraint;
        ;
    }
};

///------- QGLViewer::MouseBindingPrivate starts from here
struct QtOpenGLViewer::MouseBindingPrivate
{
    const Qt::KeyboardModifiers modifiers;
    const Qt::MouseButton button;
    const Qt::Key key;

    MouseBindingPrivate(Qt::KeyboardModifiers m, Qt::MouseButton b, Qt::Key k)
        : modifiers(m), button(b), key(k)
    {
    }

    // This sort order is used in mouseString() to display sorted mouse
    // bindings
    bool operator<(const MouseBindingPrivate &mbp) const
    {
        if (key != mbp.key)
            return key < mbp.key;
        if (modifiers != mbp.modifiers)
            return modifiers < mbp.modifiers;
        return button < mbp.button;
    }
};

///------- QGLViewer::ClickBindingPrivate starts from here
struct QtOpenGLViewer::ClickBindingPrivate
{
    const Qt::KeyboardModifiers modifiers;
    const Qt::MouseButton button;
    const bool doubleClick;
    const Qt::MouseButtons buttonsBefore; // Use with doubleClick
    const Qt::Key key;

    ClickBindingPrivate(Qt::KeyboardModifiers m, Qt::MouseButton b, bool dc,
                        Qt::MouseButtons bb, Qt::Key k)
        : modifiers(m), button(b), doubleClick(dc), buttonsBefore(bb), key(k)
    {
    }

    QKeyCombination toCombination() const
    {
        return QKeyCombination{modifiers, key};
    }

    QString toString() const
    {
        const bool existButtonsBefore = buttonsBefore != Qt::NoButton;
        auto keyModifierString = keyString(toCombination());
        if (!keyModifierString.isEmpty()) {
            // modifiers might be of the form : 'S' or 'Ctrl+S' or 'Ctrl+'. For
            // consistency, add an other '+' if needed, no spaces
            if (!keyModifierString.endsWith('+'))
                keyModifierString += "+";
        }

        return tr("%1%2%3%4%5%6",
                  "Modifier / button or wheel / double click / with "
                  "/ button / pressed")
            .arg(keyModifierString,
                 mouseButtonsString(button) +
                     (button == Qt::NoButton ? tr("Wheel", "Mouse wheel") : ""),
                 doubleClick ? tr(" double click", "Suffix after mouse button")
                             : "",
                 existButtonsBefore
                     ? tr(" with ", "As in : Left button with Ctrl pressed")
                     : "",
                 existButtonsBefore ? mouseButtonsString(buttonsBefore) : "",
                 existButtonsBefore
                     ? tr(" pressed", "As in : Left button with Ctrl pressed")
                     : "");
    }

    // This sort order is used in mouseString() to display sorted mouse
    // bindings
    bool operator<(const ClickBindingPrivate &cbp) const
    {
        if (key != cbp.key)
            return key < cbp.key;
        if (buttonsBefore != cbp.buttonsBefore)
            return buttonsBefore < cbp.buttonsBefore;
        if (modifiers != cbp.modifiers)
            return modifiers < cbp.modifiers;
        if (button != cbp.button)
            return button < cbp.button;
        return doubleClick < cbp.doubleClick;
    }
};

///------- QGLViewer::WheelBindingPrivate starts from here
struct QtOpenGLViewer::WheelBindingPrivate
{
    const Qt::KeyboardModifiers modifiers;
    const Qt::Key key;

    WheelBindingPrivate(Qt::KeyboardModifiers m, Qt::Key k)
        : modifiers(m), key(k)
    {
    }

    // This sort order is used in mouseString() to display sorted wheel
    // bindings
    bool operator<(const WheelBindingPrivate &wbp) const
    {
        if (key != wbp.key)
            return key < wbp.key;
        return modifiers < wbp.modifiers;
    }
};

///------- QGLViewer starts from here
QList<QtOpenGLViewer *> QtOpenGLViewer::QGLViewerPool_;

void QtOpenGLViewer::defaultConstructor()
{
    // Test OpenGL context
    // if (glGetString(GL_VERSION) == 0)
    // qWarning("Unable to get OpenGL version, context may not be available -
    // Check your configuration");

    int poolIndex = QtOpenGLViewer::QGLViewerPool_.indexOf(nullptr);
    setFocusPolicy(Qt::StrongFocus);

    if (poolIndex >= 0)
        QtOpenGLViewer::QGLViewerPool_.replace(poolIndex, this);
    else
        QtOpenGLViewer::QGLViewerPool_.append(this);

    camera_ = new Camera();
    setCamera(camera());

    setDefaultShortcuts();
    setDefaultMouseBindings();

    setSnapshotFileName(tr("snapshot", "Default snapshot file name"));
    initializeSnapshotFormats();
    setSnapshotCounter(0);
    setSnapshotQuality(95);

    fpsTime_.start();
    fpsCounter_ = 0;
    fps_ = 0.0;
    fpsString_ = tr("%1Hz", "Frames per seconds, in Hertz").arg("?");
    visualHint_ = 0;
    previousPathId_ = 0;
    // prevPos_ is not initialized since pos() is not meaningful here.
    // It will be set when setFullScreen(false) is called after
    // setFullScreen(true)

    // #CONNECTION# default values in initFromDOMElement()
    manipulatedFrame_ = nullptr;
    manipulatedFrameIsACamera_ = false;
    mouseGrabberIsAManipulatedFrame_ = false;
    mouseGrabberIsAManipulatedCameraFrame_ = false;
    displayMessage_ = false;
    connect(&messageTimer_, &QTimer::timeout, this,
            &QtOpenGLViewer::hideMessage);
    messageTimer_.setSingleShot(true);
    helpWidget_ = nullptr;
    setMouseGrabber(nullptr);

    setSceneRadius(1.0);
    showEntireScene();
    setStateFileName(".qglviewer.xml");

    // #CONNECTION# default values in initFromDOMElement()
    setAxisIsDrawn(false);
    setGridIsDrawn(false);
    setFPSIsDisplayed(false);
    setCameraIsEdited(false);
    setTextIsEnabled(true);
    setStereoDisplay(false);
    // Make sure move() is not called, which would call initializeGL()
    fullScreen_ = false;
    setFullScreen(false);

    animationTimerId_ = 0;
    stopAnimation();
    setAnimationPeriod(40); // 25Hz

    selectBuffer_ = nullptr;
    setSelectBufferSize(4 * 1000);
    setSelectRegionWidth(3);
    setSelectRegionHeight(3);
    setSelectedName(-1);

    bufferTextureId_ = 0;
    bufferTextureMaxU_ = 0.0;
    bufferTextureMaxV_ = 0.0;
    bufferTextureWidth_ = 0;
    bufferTextureHeight_ = 0;
    previousBufferTextureFormat_ = 0;
    previousBufferTextureInternalFormat_ = 0;
    currentlyPressedKey_ = Qt::Key(0);

    setAttribute(Qt::WA_NoSystemBackground);

    tileRegion_ = nullptr;
}

QtOpenGLViewer::QtOpenGLViewer(QWidget *parent, Qt::WindowFlags flags)
    : QOpenGLWidget(parent, flags)
{
    defaultConstructor();
}

QtOpenGLViewer::~QtOpenGLViewer()
{
    // See closeEvent comment. Destructor is called (and not closeEvent) only
    // when the widget is embedded. Hence we saveToFile here. It is however a
    // bad idea if virtual domElement() has been overloaded ! if (parent())
    // saveStateToFileForAllViewers();

    QtOpenGLViewer::QGLViewerPool_.replace(
        QtOpenGLViewer::QGLViewerPool_.indexOf(this), nullptr);

    delete camera();
    delete[] selectBuffer_;
    if (helpWidget()) {
        // Needed for Qt 4 which has no main widget.
        helpWidget()->close();
        delete helpWidget_;
    }
}

QColor QtOpenGLViewer::backgroundColor() const { return backgroundColor_; }

void QtOpenGLViewer::setBackgroundColor(const QColor &color)
{
    backgroundColor_ = color;
    glClearColor(color.redF(), color.greenF(), color.blueF(), color.alphaF());
}

QColor QtOpenGLViewer::foregroundColor() const { return foregroundColor_; }

void QtOpenGLViewer::setForegroundColor(const QColor &color)
{
    foregroundColor_ = color;
}

void QtOpenGLViewer::initializeGL()
{
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);

    // Default colors
    setForegroundColor(QColor(180, 180, 180));
    setBackgroundColor(QColor(51, 51, 51));

    // Clear the buffer where we're going to draw
    if (format().stereo()) {
        glDrawBuffer(GL_BACK_RIGHT);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glDrawBuffer(GL_BACK_LEFT);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }
    else
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Calls user defined method. Default emits a signal.
    init();

    // Give time to glInit to finish and then call setFullScreen().
    if (isFullScreen()) {
        QTimer::singleShot(100, this, &QtOpenGLViewer::delayedFullScreen);
    }
}

void QtOpenGLViewer::paintGL()
{
    if (displaysInStereo()) {
        for (int view = 1; view >= 0; --view) {
            // Clears screen, set model view matrix with shifted matrix for ith
            // buffer
            preDrawStereo(view);
            // Used defined method. Default is empty
            if (camera()->frame()->isManipulated())
                fastDraw();
            else
                draw();
            postDraw();
        }
    }
    else {
        // Clears screen, set model view matrix...
        preDraw();
        // Used defined method. Default calls draw()
        if (camera()->frame()->isManipulated())
            fastDraw();
        else
            draw();
        // Add visual hints: axis, camera, grid...
        postDraw();
    }
    emit drawFinished(true);
}

void QtOpenGLViewer::preDraw()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // GL_PROJECTION matrix
    camera()->loadProjectionMatrix();
    // GL_MODELVIEW matrix
    camera()->loadModelViewMatrix();

    emit drawNeeded();
}

void QtOpenGLViewer::postDraw()
{
    // Reset model view matrix to world coordinates origin
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    camera()->loadModelViewMatrix();
    // TODO restore model loadProjectionMatrixStereo

    // Save OpenGL state
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    // Set neutral GL state
    glDisable(GL_TEXTURE_1D);
    glDisable(GL_TEXTURE_2D);
#ifdef GL_TEXTURE_3D // OpenGL 1.2 Only...
    glDisable(GL_TEXTURE_3D);
#endif

    glDisable(GL_TEXTURE_GEN_Q);
    glDisable(GL_TEXTURE_GEN_R);
    glDisable(GL_TEXTURE_GEN_S);
    glDisable(GL_TEXTURE_GEN_T);

#ifdef GL_RESCALE_NORMAL // OpenGL 1.2 Only...
    glEnable(GL_RESCALE_NORMAL);
#endif

    glDisable(GL_COLOR_MATERIAL);
    glColor4f(foregroundColor().redF(), foregroundColor().greenF(),
              foregroundColor().blueF(), foregroundColor().alphaF());

    if (cameraIsEdited())
        camera()->drawAllPaths();

    // Pivot point, line when camera rolls, zoom region
    drawVisualHints();

    if (gridIsDrawn()) {
        glLineWidth(1.0);
        drawGrid(camera()->sceneRadius());
    }
    if (axisIsDrawn()) {
        glLineWidth(2.0);
        drawAxis(camera()->sceneRadius());
    }

    // FPS computation
    const unsigned int maxCounter = 20;
    if (++fpsCounter_ == maxCounter) {
        fps_ = 1000.0 * maxCounter / fpsTime_.restart();
        fpsString_ = tr("%1Hz", "Frames per seconds, in Hertz")
                         .arg(fps_, 0, 'f', ((fps_ < 10.0) ? 1 : 0));
        fpsCounter_ = 0;
    }

    // Restore foregroundColor
    float color[4];
    color[0] = foregroundColor().red() / 255.0f;
    color[1] = foregroundColor().green() / 255.0f;
    color[2] = foregroundColor().blue() / 255.0f;
    color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    if (FPSIsDisplayed())
        displayFPS();
    if (displayMessage_)
        drawText(10, height() - 10, message_);

    // Restore GL state
    glPopAttrib();
    glPopMatrix();
}

void QtOpenGLViewer::preDrawStereo(bool leftBuffer)
{
    // Set buffer to draw in
    // Seems that SGI and Crystal Eyes are not synchronized correctly !
    // That's why we don't draw in the appropriate buffer...
    if (!leftBuffer)
        glDrawBuffer(GL_BACK_LEFT);
    else
        glDrawBuffer(GL_BACK_RIGHT);

    // Clear the buffer where we're going to draw
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // GL_PROJECTION matrix
    camera()->loadProjectionMatrixStereo(leftBuffer);
    // GL_MODELVIEW matrix
    camera()->loadModelViewMatrixStereo(leftBuffer);

    emit drawNeeded();
}

void QtOpenGLViewer::fastDraw() { draw(); }

bool QtOpenGLViewer::cameraIsEdited() const { return cameraIsEdited_; }

void QtOpenGLViewer::setCameraIsEdited(bool edit)
{
    cameraIsEdited_ = edit;
    if (edit) {
        previousCameraZClippingCoefficient_ = camera()->zClippingCoefficient();
        // #CONNECTION# 5.0 also used in domElement() and in
        // initFromDOMElement().
        camera()->setZClippingCoefficient(5.0);
    }
    else
        camera()->setZClippingCoefficient(previousCameraZClippingCoefficient_);

    emit cameraIsEditedChanged(edit);

    update();
}

// Key bindings. 0 means not defined
void QtOpenGLViewer::setDefaultShortcuts()
{
    setShortcut(ShowWorldFrame, Qt::Key_A);
    setShortcut(ShowWorldGrid, Qt::Key_G);
    setShortcut(ShowFPS, Qt::Key_F);
    setShortcut(EnableText,
                QKeyCombination{Qt::ShiftModifier, Qt::Key_Question});
    setShortcut(ExitViewer, Qt::Key_Escape);
    setShortcut(SaveScreenshot,
                QKeyCombination{Qt::ControlModifier, Qt::Key_S});
    setShortcut(CameraMode, Qt::Key_Space);
    setShortcut(FullScreen, QKeyCombination{Qt::AltModifier, Qt::Key_Return});
    setShortcut(ToggleStereoMode, Qt::Key_S);
    setShortcut(ToggleAnimation, Qt::Key_Return);
    setShortcut(Help, Qt::Key_H);
    setShortcut(EditCamera, Qt::Key_C);
    setShortcut(MoveCameraLeft, Qt::Key_Left);
    setShortcut(MoveCameraRight, Qt::Key_Right);
    setShortcut(MoveCameraUp, Qt::Key_Up);
    setShortcut(MoveCameraDown, Qt::Key_Down);
    setShortcut(IncreaseFlySpeed, Qt::Key_Plus);
    setShortcut(DecreaseFlySpeed, Qt::Key_Minus);
    setShortcut(SnapshotToClipboard,
                QKeyCombination{Qt::ControlModifier, Qt::Key_C});

    keyboardActionDescription_[ShowFPS] =
        tr("Toggles the display of the FPS", "DISPLAY_FPS action description");
    keyboardActionDescription_[SaveScreenshot] =
        tr("Saves a screenshot", "SAVE_SCREENSHOT action description");
    keyboardActionDescription_[FullScreen] =
        tr("Toggles full screen display", "FULL_SCREEN action description");
    keyboardActionDescription_[ShowWorldFrame] =
        tr("Toggles the display of the world axis",
           "DRAW_AXIS action description");
    keyboardActionDescription_[ShowWorldGrid] = tr(
        "Toggles the display of the XY grid", "DRAW_GRID action description");
    keyboardActionDescription_[CameraMode] =
        tr("Changes camera mode (observe or fly)",
           "CAMERA_MODE action description");
    keyboardActionDescription_[ToggleStereoMode] =
        tr("Toggles stereo display", "STEREO action description");
    keyboardActionDescription_[Help] =
        tr("Opens this help window", "HELP action description");
    keyboardActionDescription_[ToggleAnimation] =
        tr("Starts/stops the animation", "ANIMATION action description");
    keyboardActionDescription_[EditCamera] =
        tr("Toggles camera paths display",
           "EDIT_CAMERA action description"); // TODO change
    keyboardActionDescription_[EnableText] =
        tr("Toggles the display of the text", "ENABLE_TEXT action description");
    keyboardActionDescription_[ExitViewer] =
        tr("Exits program", "EXIT_VIEWER action description");
    keyboardActionDescription_[MoveCameraLeft] =
        tr("Moves camera left", "MOVE_CAMERA_LEFT action description");
    keyboardActionDescription_[MoveCameraRight] =
        tr("Moves camera right", "MOVE_CAMERA_RIGHT action description");
    keyboardActionDescription_[MoveCameraUp] =
        tr("Moves camera up", "MOVE_CAMERA_UP action description");
    keyboardActionDescription_[MoveCameraDown] =
        tr("Moves camera down", "MOVE_CAMERA_DOWN action description");
    keyboardActionDescription_[IncreaseFlySpeed] =
        tr("Increases fly speed", "INCREASE_FLYSPEED action description");
    keyboardActionDescription_[DecreaseFlySpeed] =
        tr("Decreases fly speed", "DECREASE_FLYSPEED action description");
    keyboardActionDescription_[SnapshotToClipboard] =
        tr("Copies a snapshot to clipboard",
           "SNAPSHOT_TO_CLIPBOARD action description");

    // Keyframe related shortcuts
    setPathKey(Qt::Key_F1, 1);
    setPathKey(Qt::Key_F2, 2);
    setPathKey(Qt::Key_F3, 3);
    setPathKey(Qt::Key_F4, 4);
    setPathKey(Qt::Key_F5, 5);
    setPathKey(Qt::Key_F6, 6);
    setPathKey(Qt::Key_F7, 7);
    setPathKey(Qt::Key_F8, 8);
    setPathKey(Qt::Key_F9, 9);
    setPathKey(Qt::Key_F10, 10);
    setPathKey(Qt::Key_F11, 11);
    setPathKey(Qt::Key_F12, 12);

    setAddKeyFrameKeyboardModifiers(Qt::AltModifier);
    setPlayPathKeyboardModifiers(Qt::NoModifier);
}

void QtOpenGLViewer::setDefaultMouseBindings()
{
    constexpr auto cameraKeyboardModifiers = Qt::NoModifier;
    constexpr auto frameKeyboardModifiers = Qt::ControlModifier;

    for (int handler = 0; handler < 2; ++handler) {
        MouseHandler mh = static_cast<MouseHandler>(handler);
        Qt::KeyboardModifiers modifiers =
            (mh == FRAME) ? frameKeyboardModifiers : cameraKeyboardModifiers;

        setMouseBinding(modifiers, Qt::LeftButton, mh, Rotate);
        setMouseBinding(modifiers, Qt::MiddleButton, mh, Zoom);
        setMouseBinding(modifiers, Qt::RightButton, mh, Translate);

        setMouseBinding(Qt::Key_R, modifiers, Qt::LeftButton, mh, ScreenRotate);

        setWheelBinding(modifiers, mh, Zoom);
    }

    setMouseBinding(Qt::ShiftModifier, Qt::MiddleButton, CAMERA, ZoomOnRegion);
    setMouseBinding(Qt::ShiftModifier, Qt::LeftButton, Select);
    setMouseBinding(Qt::ShiftModifier, Qt::RightButton, RAP_FROM_PIXEL);
    // DoubleClick
    setMouseBinding(Qt::NoModifier, Qt::LeftButton, AlignCamera, true);
    setMouseBinding(Qt::NoModifier, Qt::MiddleButton, ShowEntireScene, true);
    setMouseBinding(Qt::NoModifier, Qt::RightButton, CenterScene, true);

    setMouseBinding(frameKeyboardModifiers, Qt::LeftButton, AlignFrame, true);
    // middle double click makes no sense for manipulated frame
    setMouseBinding(frameKeyboardModifiers, Qt::RightButton, CenterFrame, true);

    // A c t i o n s   w i t h   k e y   m o d i f i e r s
    setMouseBinding(Qt::Key_Z, Qt::NoModifier, Qt::LeftButton, ZoomOnPixel);
    setMouseBinding(Qt::Key_Z, Qt::NoModifier, Qt::RightButton, ZoomToFit);
}

viewer::Camera *QtOpenGLViewer::camera() const { return camera_; }

void QtOpenGLViewer::setCamera(Camera *const camera)
{
    if (!camera) {
        return;
    }

    // Disconnect current camera from this viewer.
    disconnect(this->camera()->frame(), &ManipulatedFrame::manipulated, this,
               qOverload<>(&QtOpenGLViewer::update));
    disconnect(this->camera()->frame(), &ManipulatedFrame::spun, this,
               qOverload<>(&QtOpenGLViewer::update));
    disconnect(screen(), &QScreen::physicalDotsPerInchChanged, this->camera(),
               &Camera::setDevicePixelRatio);
    connectAllCameraKFIInterpolatedSignals(false);

    camera_ = camera;

    camera->setSceneRadius(sceneRadius());
    camera->setSceneCenter(sceneCenter());
    camera->setScreenWidthAndHeight(width(), height());
    camera->setDevicePixelRatio(screen()->devicePixelRatio());

    // Connect camera frame to this viewer.
    connect(camera->frame(), &ManipulatedFrame::manipulated, this,
            qOverload<>(&QtOpenGLViewer::update));
    connect(camera->frame(), &ManipulatedFrame::spun, this,
            qOverload<>(&QtOpenGLViewer::update));
    connect(screen(), &QScreen::physicalDotsPerInchChanged, this->camera(),
            &Camera::setDevicePixelRatio);
    connectAllCameraKFIInterpolatedSignals();

    previousCameraZClippingCoefficient_ =
        this->camera()->zClippingCoefficient();
}

void QtOpenGLViewer::connectAllCameraKFIInterpolatedSignals(bool connection)
{
    for (auto it = camera()->kfi_.begin(); it != camera()->kfi_.end(); ++it) {
        if (connection) {
            connect(camera()->keyFrameInterpolator(it.key()),
                    &KeyFrameInterpolator::interpolated, this,
                    qOverload<>(&QtOpenGLViewer::update));
        }
        else {
            disconnect(camera()->keyFrameInterpolator(it.key()),
                       &KeyFrameInterpolator::interpolated, this,
                       qOverload<>(&QtOpenGLViewer::update));
        }
    }

    if (connection) {
        connect(camera()->interpolationKfi_,
                &KeyFrameInterpolator::interpolated, this,
                qOverload<>(&QtOpenGLViewer::update));
    }
    else {
        disconnect(camera()->interpolationKfi_,
                   &KeyFrameInterpolator::interpolated, this,
                   qOverload<>(&QtOpenGLViewer::update));
    }
}

void QtOpenGLViewer::drawLight(GLenum light, qreal scale) const
{
    static GLUquadric *quadric = gluNewQuadric();

    const qreal length = sceneRadius() / 5.0 * scale;

    GLboolean lightIsOn;
    glGetBooleanv(light, &lightIsOn);

    if (lightIsOn) {
        // All light values are given in eye coordinates
        glPushMatrix();
        glLoadIdentity();

        float color[4];
        glGetLightfv(light, GL_DIFFUSE, color);
        glColor4fv(color);

        float pos[4];
        glGetLightfv(light, GL_POSITION, pos);

        if (static_cast<double>(pos[3]) != 0.0) {
            glTranslatef(pos[0] / pos[3], pos[1] / pos[3], pos[2] / pos[3]);

            GLfloat cutOff;
            glGetLightfv(light, GL_SPOT_CUTOFF, &cutOff);
            if (static_cast<double>(cutOff) != 180.0) {
                GLfloat dir[4];
                glGetLightfv(light, GL_SPOT_DIRECTION, dir);
                glMultMatrixd(Quaternion(Vec(0, 0, 1), Vec(dir)).matrix());
                QtOpenGLViewer::drawArrow(length);
                gluCylinder(quadric, 0.0,
                            0.7 * length *
                                sin(static_cast<double>(cutOff) * M_PI / 180.0),
                            0.7 * length *
                                cos(static_cast<double>(cutOff) * M_PI / 180.0),
                            12, 1);
            }
            else
                gluSphere(quadric, 0.2 * length, 10, 10);
        }
        else {
            // Directional light.
            Vec dir(static_cast<double>(pos[0]), static_cast<double>(pos[1]),
                    static_cast<double>(pos[2]));
            dir.normalize();
            Frame fr = Frame(
                camera()->cameraCoordinatesOf(
                    4.0 * length * camera()->frame()->inverseTransformOf(dir)),
                Quaternion(Vec(0, 0, -1), dir));
            glMultMatrixd(fr.matrix());
            drawArrow(length);
        }

        glPopMatrix();
    }
}

void QtOpenGLViewer::renderText(int x, int y, const QString &str,
                                const QFont &font)
{
    // Retrieve last OpenGL color to use as a font color
    GLdouble glColor[4];
    glGetDoublev(GL_CURRENT_COLOR, glColor);
    QColor fontColor = QColor(255 * glColor[0], 255 * glColor[1],
                              255 * glColor[2], 255 * glColor[3]);

    // Render text
    QPainter painter(this);
    painter.setPen(fontColor);
    painter.setFont(font);
    painter.drawText(x, y, str);
    painter.end();
}

void QtOpenGLViewer::renderText(double x, double y, double z,
                                const QString &str, const QFont &font)
{
    const Vec proj = camera_->projectedCoordinatesOf(Vec(x, y, z));
    renderText(proj.x, proj.y, str, font);
}

void QtOpenGLViewer::setSceneBoundingBox(const viewer::Vec &min,
                                         const viewer::Vec &max)
{
    camera()->setSceneBoundingBox(min, max);
}

void QtOpenGLViewer::showEntireScene()
{
    camera()->showEntireScene();
    update();
}

bool QtOpenGLViewer::axisIsDrawn() const { return axisIsDrawn_; }

void QtOpenGLViewer::setAxisIsDrawn(bool draw)
{
    axisIsDrawn_ = draw;
    emit axisIsDrawnChanged(draw);
    update();
}

bool QtOpenGLViewer::gridIsDrawn() const { return gridIsDrawn_; }

void QtOpenGLViewer::setGridIsDrawn(bool draw)
{
    gridIsDrawn_ = draw;
    emit gridIsDrawnChanged(draw);
    update();
}

bool QtOpenGLViewer::FPSIsDisplayed() const { return FPSIsDisplayed_; }

void QtOpenGLViewer::setFPSIsDisplayed(bool display)
{
    FPSIsDisplayed_ = display;
    emit FPSIsDisplayedChanged(display);
    update();
}

bool QtOpenGLViewer::textIsEnabled() const { return textIsEnabled_; }

void QtOpenGLViewer::setTextIsEnabled(bool enable)
{
    textIsEnabled_ = enable;
    emit textIsEnabledChanged(enable);
    update();
}

qreal QtOpenGLViewer::sceneRadius() const { return camera()->sceneRadius(); }

void QtOpenGLViewer::setSceneRadius(qreal radius)
{
    camera()->setSceneRadius(radius);
}

viewer::Vec QtOpenGLViewer::sceneCenter() const
{
    return camera()->sceneCenter();
}

void QtOpenGLViewer::setSceneCenter(const viewer::Vec &center)
{
    camera()->setSceneCenter(center);
}

void QtOpenGLViewer::drawText(int x, int y, const QString &text,
                              const QFont &fnt)
{
    if (!textIsEnabled())
        return;

    if (tileRegion_) {
        renderText(int((x - tileRegion_->xMin) * width() /
                       (tileRegion_->xMax - tileRegion_->xMin)),
                   int((y - tileRegion_->yMin) * height() /
                       (tileRegion_->yMax - tileRegion_->yMin)),
                   text, scaledFont(fnt));
    }
    else
        renderText(x, y, text, fnt);
}

void QtOpenGLViewer::displayMessage(const QString &message, int delay)
{
    message_ = message;
    displayMessage_ = true;
    // Was set to single shot in defaultConstructor.
    messageTimer_.start(delay);
    if (textIsEnabled())
        update();
}

void QtOpenGLViewer::hideMessage()
{
    displayMessage_ = false;
    if (textIsEnabled()) {
        update();
    }
}

void QtOpenGLViewer::displayFPS()
{
    drawText(10,
             int(1.5 * ((QApplication::font().pixelSize() > 0)
                            ? QApplication::font().pixelSize()
                            : QApplication::font().pointSize())),
             fpsString_);
}

void QtOpenGLViewer::startScreenCoordinatesSystem(bool upward) const
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    if (tileRegion_)
        if (upward)
            glOrtho(tileRegion_->xMin, tileRegion_->xMax, tileRegion_->yMin,
                    tileRegion_->yMax, 0.0, -1.0);
        else
            glOrtho(tileRegion_->xMin, tileRegion_->xMax, tileRegion_->yMax,
                    tileRegion_->yMin, 0.0, -1.0);
    else if (upward)
        glOrtho(0, width(), 0, height(), 0.0, -1.0);
    else
        glOrtho(0, width(), height(), 0, 0.0, -1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
}

void QtOpenGLViewer::stopScreenCoordinatesSystem() const
{
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

void QtOpenGLViewer::timerEvent(QTimerEvent *)
{
    if (animationIsStarted()) {
        animate();
        update();
    }
}

int QtOpenGLViewer::animationPeriod() const { return animationPeriod_; }
void QtOpenGLViewer::setAnimationPeriod(int period)
{
    animationPeriod_ = period;
}

bool QtOpenGLViewer::animationIsStarted() const { return animationStarted_; }

void QtOpenGLViewer::startAnimation()
{
    animationTimerId_ = startTimer(animationPeriod());
    animationStarted_ = true;
}

void QtOpenGLViewer::stopAnimation()
{
    animationStarted_ = false;
    if (animationTimerId_ != 0)
        killTimer(animationTimerId_);
}

const QList<QtOpenGLViewer *> &QtOpenGLViewer::QGLViewerPool()
{
    return QtOpenGLViewer::QGLViewerPool_;
}

int QtOpenGLViewer::QGLViewerIndex(const QtOpenGLViewer *const viewer)
{
    return QtOpenGLViewer::QGLViewerPool_.indexOf(
        const_cast<QtOpenGLViewer *>(viewer));
}

void QtOpenGLViewer::closeEvent(QCloseEvent *e)
{
    // When the user clicks on the window close (x) button:
    // - If the viewer is a top level window, closeEvent is called and then
    // saves to file. - Otherwise, nothing happen s:( When the user press the
    // EXIT_VIEWER keyboard shortcut: - If the viewer is a top level window,
    // saveStateToFile() is also called - Otherwise, closeEvent is NOT called
    // and keyPressEvent does the job.

    /* After tests:
    E : Embedded widget
    N : Widget created with new
    C : closeEvent called
    D : destructor called

    E	N	C	D
    y	y
    y	n		y
    n	y	y
    n	n	y	y

    closeEvent is called iif the widget is NOT embedded.

    Destructor is called iif the widget is created on the stack
    or if widget (resp. parent if embedded) is created with WDestructiveClose
    flag.

    closeEvent always before destructor.

    Close using qApp->closeAllWindows or (x) is identical.
    */

    // #CONNECTION# Also done for EXIT_VIEWER in keyPressEvent().
    saveStateToFile();
    QOpenGLWidget::closeEvent(e);
}

void QtOpenGLViewer::select(const QMouseEvent *event)
{
    // For those who don't derive but rather rely on the signal-slot mechanism.
    emit pointSelected(event);
    select(event->pos());
}

void QtOpenGLViewer::select(const QPoint &point)
{
    beginSelection(point);
    drawWithNames();
    endSelection(point);
    postSelection(point);
}

void QtOpenGLViewer::beginSelection(const QPoint &point)
{
    // Make OpenGL context current (may be needed with several viewers ?)
    makeCurrent();

    // Prepare the selection mode
    glSelectBuffer(selectBufferSize(), selectBuffer());
    glRenderMode(GL_SELECT);
    glInitNames();

    // Loads the matrices
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    static GLint viewport[4];
    camera()->getViewport(viewport);
    gluPickMatrix(point.x(), point.y(), selectRegionWidth(),
                  selectRegionHeight(), viewport);

    // loadProjectionMatrix() first resets the GL_PROJECTION matrix with a
    // glLoadIdentity(). The false parameter prevents this and hence multiplies
    // the matrices.
    camera()->loadProjectionMatrix(false);
    // Reset the original (world coordinates) modelview matrix
    camera()->loadModelViewMatrix();
}

void QtOpenGLViewer::endSelection(const QPoint &point)
{
    Q_UNUSED(point)

    // Flush GL buffers
    glFlush();

    // Get the number of objects that were seen through the pick matrix frustum.
    // Reset GL_RENDER mode.
    GLint nbHits = glRenderMode(GL_RENDER);

    if (nbHits <= 0)
        setSelectedName(-1);
    else {
        // Interpret results: each object created 4 values in the
        // selectBuffer(). selectBuffer[4*i+1] is the object minimum depth
        // value, while selectBuffer[4*i+3] is the id pushed on the stack. Of
        // all the objects that were projected in the pick region, we select the
        // closest one (zMin comparison). This code needs to be modified if you
        // use several stack levels. See glSelectBuffer() man page.
        GLuint zMin = (selectBuffer())[1];
        setSelectedName(int((selectBuffer())[3]));
        for (int i = 1; i < nbHits; ++i) {
            if ((selectBuffer())[4 * i + 1] < zMin) {
                zMin = (selectBuffer())[4 * i + 1];
                setSelectedName(int((selectBuffer())[4 * i + 3]));
            }
        }
    }
}

int QtOpenGLViewer::selectedName() const { return selectedObjectId_; }

void QtOpenGLViewer::setSelectedName(int id) { selectedObjectId_ = id; }

int QtOpenGLViewer::selectBufferSize() const { return selectBufferSize_; }

void QtOpenGLViewer::setSelectBufferSize(int size)
{
    if (selectBuffer_) {
        delete[] selectBuffer_;
    }
    selectBufferSize_ = size;
    selectBuffer_ = new GLuint[selectBufferSize()];
}

int QtOpenGLViewer::selectRegionWidth() const { return selectRegionWidth_; }

void QtOpenGLViewer::setSelectRegionWidth(int width)
{
    selectRegionWidth_ = width;
}

int QtOpenGLViewer::selectRegionHeight() const { return selectRegionHeight_; }

void QtOpenGLViewer::setSelectRegionHeight(int height)
{
    selectRegionHeight_ = height;
}

GLuint *QtOpenGLViewer::selectBuffer() { return selectBuffer_; }

void QtOpenGLViewer::performClickAction(ClickAction ca,
                                        const QMouseEvent *const e)
{
    // Note: action that need it should call update().
    switch (ca) {
            // # CONNECTION setMouseBinding prevents adding NO_CLICK_ACTION in
            // clickBinding_ This case should hence not be possible. Prevents
            // unused case warning.
        case NoClick:
            break;
        case ZoomOnPixel:
            camera()->interpolateToZoomOnPixel(e->pos());
            break;
        case ZoomToFit:
            camera()->interpolateToFitScene();
            break;
        case Select:
            select(e);
            update();
            break;
        case RAP_FROM_PIXEL:
            makeCurrent();
            if (!camera()->setPivotPointFromPixel(e->pos())) {
                camera()->setPivotPoint(sceneCenter());
            }
            setVisualHintsMask(1);
            update();
            break;
        case RAP_IS_CENTER:
            camera()->setPivotPoint(sceneCenter());
            setVisualHintsMask(1);
            update();
            break;
        case CenterFrame:
            if (manipulatedFrame()) {
                manipulatedFrame()->projectOnLine(camera()->position(),
                                                  camera()->viewDirection());
            }
            break;
        case CenterScene:
            camera()->centerScene();
            break;
        case ShowEntireScene:
            camera()->showEntireScene();
            break;
        case AlignFrame:
            if (manipulatedFrame()) {
                manipulatedFrame()->alignWithFrame(camera()->frame());
            }
            break;
        case AlignCamera:
            Frame *frame = new Frame();
            frame->setTranslation(camera()->pivotPoint());
            camera()->frame()->alignWithFrame(frame, true);
            delete frame;
            break;
    }
}

void QtOpenGLViewer::mousePressEvent(QMouseEvent *e)
{
    // #CONNECTION# mouseDoubleClickEvent has the same structure
    // #CONNECTION# mouseString() concatenates bindings description in inverse
    //  order.
    ClickBindingPrivate cbp(
        e->modifiers(), e->button(), false,
        static_cast<Qt::MouseButtons>((e->buttons() & ~(e->button()))),
        currentlyPressedKey_);

    if (clickBinding_.contains(cbp)) {
        performClickAction(clickBinding_[cbp], e);
    }
    else if (mouseGrabber()) {
        if (mouseGrabberIsAManipulatedFrame_) {
            for (auto it = mouseBinding_.begin(), end = mouseBinding_.end();
                 it != end; ++it)
                if ((it.value().handler == FRAME) &&
                    (it.key().button == e->button())) {
                    auto *mf = dynamic_cast<ManipulatedFrame *>(mouseGrabber());
                    if (mouseGrabberIsAManipulatedCameraFrame_) {
                        mf->ManipulatedFrame::startAction(
                            it.value().action, it.value().withConstraint);
                        mf->ManipulatedFrame::mousePressEvent(e, camera());
                    }
                    else {
                        mf->startAction(it.value().action,
                                        it.value().withConstraint);
                        mf->mousePressEvent(e, camera());
                    }
                    break;
                }
        }
        else {
            mouseGrabber()->mousePressEvent(e, camera());
        }
        update();
    }
    else {
        // #CONNECTION# wheelEvent has the same structure
        const MouseBindingPrivate mbp(e->modifiers(), e->button(),
                                      currentlyPressedKey_);

        if (mouseBinding_.contains(mbp)) {
            MouseActionPrivate map = mouseBinding_[mbp];
            switch (map.handler) {
                case CAMERA:
                    camera()->frame()->startAction(map.action,
                                                   map.withConstraint);
                    camera()->frame()->mousePressEvent(e, camera());
                    break;
                case FRAME:
                    if (manipulatedFrame()) {
                        if (manipulatedFrameIsACamera_) {
                            manipulatedFrame()->ManipulatedFrame::startAction(
                                map.action, map.withConstraint);
                            manipulatedFrame()
                                ->ManipulatedFrame::mousePressEvent(e,
                                                                    camera());
                        }
                        else {
                            manipulatedFrame()->startAction(map.action,
                                                            map.withConstraint);
                            manipulatedFrame()->mousePressEvent(e, camera());
                        }
                    }
                    break;
            }
            if (map.action == ScreenRotate)
                // Display visual hint line
                update();
        }
        else
            e->ignore();
    }
}

void QtOpenGLViewer::mouseMoveEvent(QMouseEvent *e)
{
    if (mouseGrabber()) {
        const auto pos = e->pos();
        mouseGrabber()->checkIfGrabsMouse(pos.x(), pos.y(), camera());
        if (mouseGrabber()->grabsMouse()) {
            if (mouseGrabberIsAManipulatedCameraFrame_) {
                (dynamic_cast<ManipulatedFrame *>(mouseGrabber()))
                    ->ManipulatedFrame::mouseMoveEvent(e, camera());
            }
            else {
                mouseGrabber()->mouseMoveEvent(e, camera());
            }
        }
        else {
            setMouseGrabber(nullptr);
        }
        update();
    }

    if (!mouseGrabber()) {
        // #CONNECTION# mouseReleaseEvent has the same structure
        if (camera()->frame()->isManipulated()) {
            camera()->frame()->mouseMoveEvent(e, camera());
            // #CONNECTION# manipulatedCameraFrame::mouseMoveEvent specific if
            // at the beginning
            if (camera()->frame()->action_ == ZoomOnRegion)
                update();
        }
        else // !
            if ((manipulatedFrame()) && (manipulatedFrame()->isManipulated()))
                if (manipulatedFrameIsACamera_)
                    manipulatedFrame()->ManipulatedFrame::mouseMoveEvent(
                        e, camera());
                else
                    manipulatedFrame()->mouseMoveEvent(e, camera());
            else if (hasMouseTracking()) {
                for (auto *mg : MouseGrabber::MouseGrabberPool()) {
                    const auto pos = e->pos();
                    mg->checkIfGrabsMouse(pos.x(), pos.y(), camera());
                    if (mg->grabsMouse()) {
                        setMouseGrabber(mg);
                        // Check that MouseGrabber is not disabled
                        if (mouseGrabber() == mg) {
                            update();
                            break;
                        }
                    }
                }
            }
    }
}

void QtOpenGLViewer::mouseReleaseEvent(QMouseEvent *e)
{
    if (mouseGrabber()) {
        if (mouseGrabberIsAManipulatedCameraFrame_) {
            (dynamic_cast<ManipulatedFrame *>(mouseGrabber()))
                ->ManipulatedFrame::mouseReleaseEvent(e, camera());
        }
        else {
            mouseGrabber()->mouseReleaseEvent(e, camera());
        }

        const auto pos = e->pos();
        mouseGrabber()->checkIfGrabsMouse(pos.x(), pos.y(), camera());
        if (!(mouseGrabber()->grabsMouse())) {
            setMouseGrabber(nullptr);
        }
        // update();
    }
    else {
        // #CONNECTION# mouseMoveEvent has the same structure
        if (camera()->frame()->isManipulated()) {
            camera()->frame()->mouseReleaseEvent(e, camera());
        }
        else if ((manipulatedFrame()) &&
                 (manipulatedFrame()->isManipulated())) {
            if (manipulatedFrameIsACamera_) {
                manipulatedFrame()->ManipulatedFrame::mouseReleaseEvent(
                    e, camera());
            }
            else {
                manipulatedFrame()->mouseReleaseEvent(e, camera());
            }
        }
        else {
            e->ignore();
        }
    }

    // Not absolutely needed (see above commented code for the optimal version),
    // but may reveal useful for specific applications.
    update();
}

void QtOpenGLViewer::wheelEvent(QWheelEvent *e)
{
    if (mouseGrabber()) {
        if (mouseGrabberIsAManipulatedFrame_) {
            for (auto it = wheelBinding_.begin(); it != wheelBinding_.end();
                 ++it) {
                if (it.value().handler == FRAME) {
                    auto *mf = dynamic_cast<ManipulatedFrame *>(mouseGrabber());
                    if (mouseGrabberIsAManipulatedCameraFrame_) {
                        mf->ManipulatedFrame::startAction(
                            it.value().action, it.value().withConstraint);
                        mf->ManipulatedFrame::wheelEvent(e, camera());
                    }
                    else {
                        mf->startAction(it.value().action,
                                        it.value().withConstraint);
                        mf->wheelEvent(e, camera());
                    }
                    break;
                }
            }
        }
        else {
            mouseGrabber()->wheelEvent(e, camera());
        }
        update();
    }
    else {
        // #CONNECTION# mousePressEvent has the same structure
        WheelBindingPrivate wbp(e->modifiers(), currentlyPressedKey_);

        if (wheelBinding_.contains(wbp)) {
            MouseActionPrivate map = wheelBinding_[wbp];
            switch (map.handler) {
                case CAMERA: {
                    camera()->frame()->startAction(map.action,
                                                   map.withConstraint);
                    camera()->frame()->wheelEvent(e, camera());
                } break;
                case FRAME: {
                    if (manipulatedFrame()) {
                        if (manipulatedFrameIsACamera_) {
                            manipulatedFrame()->ManipulatedFrame::startAction(
                                map.action, map.withConstraint);
                            manipulatedFrame()->ManipulatedFrame::wheelEvent(
                                e, camera());
                        }
                        else {
                            manipulatedFrame()->startAction(map.action,
                                                            map.withConstraint);
                            manipulatedFrame()->wheelEvent(e, camera());
                        }
                    }
                } break;
                default:
                    break;
            }
        }
        else {
            e->ignore();
        }
    }
}

void QtOpenGLViewer::mouseDoubleClickEvent(QMouseEvent *e)
{
    // #CONNECTION# mousePressEvent has the same structure
    ClickBindingPrivate cbp(
        e->modifiers(), e->button(), true,
        static_cast<Qt::MouseButtons>(e->buttons() & ~(e->button())),
        currentlyPressedKey_);
    if (clickBinding_.contains(cbp))
        performClickAction(clickBinding_[cbp], e);
    else if (mouseGrabber())
        mouseGrabber()->mouseDoubleClickEvent(e, camera());
    else
        e->ignore();
}

bool QtOpenGLViewer::displaysInStereo() const { return stereo_; }

void QtOpenGLViewer::setStereoDisplay(bool stereo)
{
    if (format().stereo()) {
        stereo_ = stereo;
        if (!displaysInStereo()) {
            glDrawBuffer(GL_BACK_LEFT);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glDrawBuffer(GL_BACK_RIGHT);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        }

        emit stereoChanged(stereo_);

        update();
    }
    else if (stereo) {
        QMessageBox::warning(
            this, tr("Stereo not supported", "Message box window title"),
            tr("Stereo is not supported on this display."));
    }
    else {
        stereo_ = false;
    }
}

bool QtOpenGLViewer::isFullScreen() const { return fullScreen_; }

void QtOpenGLViewer::setFullScreen(bool fullScreen)
{
    if (fullScreen_ == fullScreen) {
        return;
    }

    fullScreen_ = fullScreen;

    QWidget *tlw = topLevelWidget();

    if (isFullScreen()) {
        prevPos_ = topLevelWidget()->pos();
        tlw->showFullScreen();
        tlw->move(0, 0);
    }
    else {
        tlw->showNormal();
        tlw->move(prevPos_);
    }
}

viewer::MouseGrabber *QtOpenGLViewer::mouseGrabber() const
{
    return mouseGrabber_;
}

void QtOpenGLViewer::setMouseGrabber(MouseGrabber *mouseGrabber)
{
    if (!mouseGrabberIsEnabled(mouseGrabber))
        return;

    mouseGrabber_ = mouseGrabber;

    mouseGrabberIsAManipulatedFrame_ =
        dynamic_cast<ManipulatedFrame *>(mouseGrabber);
    mouseGrabberIsAManipulatedCameraFrame_ =
        dynamic_cast<ManipulatedCameraFrame *>(mouseGrabber) &&
        (mouseGrabber != camera()->frame());
    emit mouseGrabberChanged(mouseGrabber);
}

bool QtOpenGLViewer::mouseGrabberIsEnabled(
    const viewer::MouseGrabber *const mouseGrabber)
{
    return !disabledMouseGrabbers_.contains(
        reinterpret_cast<size_t>(mouseGrabber));
}

void QtOpenGLViewer::setMouseGrabberIsEnabled(
    const viewer::MouseGrabber *const mouseGrabber, bool enabled)
{
    if (enabled)
        disabledMouseGrabbers_.remove(reinterpret_cast<size_t>(mouseGrabber));
    else
        disabledMouseGrabbers_[reinterpret_cast<size_t>(mouseGrabber)];
}

qreal QtOpenGLViewer::aspectRatio() const
{
    return width() / static_cast<qreal>(height());
}

qreal QtOpenGLViewer::currentFPS() const { return fps_; }

void QtOpenGLViewer::setMouseBindingDescription(
    Qt::Key key, Qt::KeyboardModifiers modifiers, Qt::MouseButton button,
    QString description, bool doubleClick, Qt::MouseButtons buttonsBefore)
{
    ClickBindingPrivate cbp(modifiers, button, doubleClick, buttonsBefore, key);

    if (description.isEmpty())
        mouseDescription_.remove(cbp);
    else
        mouseDescription_[cbp] = description;
}

QString QtOpenGLViewer::mouseString() const
{
    QString text("<font color=\"black\"><center><table border=\"0\" "
                 "cellspacing=\"0\" cellpadding=\"4\">\n");

    text += QString("<tr bgcolor=\"#aaaacc\"><th align=\"center\">%1</th><th "
                    "align=\"center\">%2</th></tr>\n")
                .arg(tr("Button(s)",
                        "Buttons column header in help window mouse tab"),
                     tr("Description",
                        "Description column header in help window mouse tab"));

    QMap<ClickBindingPrivate, QString> mouseBinding;

    // User-defined mouse bindings come first.
    for (auto it = mouseDescription_.begin(), end = mouseDescription_.end();
         it != end; ++it) {
        mouseBinding[it.key()] = it.value();
    }

    for (auto it = mouseBinding.begin(), end = mouseBinding.end(); it != end;
         ++it) {
        // Should not be needed (see setMouseBindingDescription())
        if (it.value().isNull())
            continue;

        text += tableLine(it.key().toString(), it.value());
    }

    // Optional separator line
    if (!mouseBinding.isEmpty()) {
        mouseBinding.clear();
        text +=
            QString("<tr bgcolor=\"#aaaacc\"><td colspan=2>%1</td></tr>\n")
                .arg(tr("Standard mouse bindings", "In help window mouse tab"));
    }

    // Then concatenates the descriptions of wheelBinding_, mouseBinding_ and
    // clickBinding_. The order is significant and corresponds to the priorities
    // set in mousePressEvent() (reverse priority order, last one overwrites
    // previous) #CONNECTION# mousePressEvent() order
    for (auto it = mouseBinding_.begin(), end = mouseBinding_.end(); it != end;
         ++it) {
        ClickBindingPrivate cbp(it.key().modifiers, it.key().button, false,
                                Qt::NoButton, it.key().key);

        QString text = mouseActionString(it.value().action);

        if (!text.isNull()) {
            switch (it.value().handler) {
                case CAMERA:
                    text += " " + tr("camera", "Suffix after action");
                    break;
                case FRAME:
                    text +=
                        " " + tr("manipulated frame", "Suffix after action");
                    break;
            }
            if (!(it.value().withConstraint))
                text += "*";
        }
        mouseBinding[cbp] = text;
    }

    for (auto it = wheelBinding_.begin(), end = wheelBinding_.end(); it != end;
         ++it) {
        ClickBindingPrivate cbp(it.key().modifiers, Qt::NoButton, false,
                                Qt::NoButton, it.key().key);

        QString text = mouseActionString(it.value().action);

        if (!text.isNull()) {
            switch (it.value().handler) {
                case CAMERA:
                    text += " " + tr("camera", "Suffix after action");
                    break;
                case FRAME:
                    text +=
                        " " + tr("manipulated frame", "Suffix after action");
                    break;
            }
            if (!(it.value().withConstraint))
                text += "*";
        }

        mouseBinding[cbp] = text;
    }

    for (auto it = clickBinding_.begin(), end = clickBinding_.end(); it != end;
         ++it) {
        mouseBinding[it.key()] = clickActionString(it.value());
    }

    for (auto it = mouseBinding.begin(), end = mouseBinding.end(); it != end;
         ++it) {
        if (it.value().isNull())
            continue;

        text += tableLine(it.key().toString(), it.value());
    }

    text += "</table></center></font>";

    return text;
}

void QtOpenGLViewer::setKeyDescription(int key, const QString &description)
{
    if (description.isEmpty())
        keyDescription_.remove(key);
    else
        keyDescription_[key] = description;
}

QString QtOpenGLViewer::cameraPathKeysString() const
{
    if (pathIndex_.isEmpty()) {
        return {};
    }

    QVector<Qt::Key> keys;
    keys.reserve(pathIndex_.count());
    for (auto it = pathIndex_.begin(), end = pathIndex_.end(); it != end;
         ++it) {
        keys.push_back(it.key());
    }
    std::sort(keys.begin(), keys.end());

    auto it = keys.begin(), end = keys.end();
    QString res = keyString(*it);

    const int maxDisplayedKeys = 6;
    int nbDisplayedKeys = 0;
    Qt::Key previousKey = (*it);
    int state = 0;
    ++it;
    while ((it != end) && (nbDisplayedKeys < maxDisplayedKeys - 1)) {
        switch (state) {
            case 0:
                if ((*it) == previousKey + 1)
                    state++;
                else {
                    res += ", " + keyString(*it);
                    nbDisplayedKeys++;
                }
                break;
            case 1:
                if ((*it) == previousKey + 1)
                    state++;
                else {
                    res += ", " + keyString(previousKey);
                    res += ", " + keyString(*it);
                    nbDisplayedKeys += 2;
                    state = 0;
                }
                break;
            default:
                if ((*it) != previousKey + 1) {
                    res += ".." + keyString(previousKey);
                    res += ", " + keyString(*it);
                    nbDisplayedKeys += 2;
                    state = 0;
                }
                break;
        }
        previousKey = *it;
        ++it;
    }

    if (state == 1)
        res += ", " + keyString(previousKey);
    if (state == 2)
        res += ".." + keyString(previousKey);
    if (it != end)
        res += "...";

    return res;
}

QString QtOpenGLViewer::keyboardString() const
{
    QString text("<font color=\"black\"><center><table border=\"0\" "
                 "cellspacing=\"0\" cellpadding=\"4\">\n");
    text +=
        QString("<tr bgcolor=\"#aaaacc\"><th align=\"center\">%1</th><th "
                "align=\"center\">%2</th></tr>\n")
            .arg(tr("Key(s)", "Keys column header in help window mouse tab"),
                 tr("Description",
                    "Description column header in help window mouse tab"));

    QMap<int, QString> keyDescriptions;

    // 1 - User defined key descriptions
    for (auto it = keyDescription_.begin(), end = keyDescription_.end();
         it != end; ++it) {
        keyDescriptions[it.key()] = it.value();
    }

    // Add to text in sorted order
    for (auto it = keyDescriptions.begin(), end = keyDescriptions.end();
         it != end; ++it)
        text += tableLine(keyString(it.key()), it.value());

    // 2 - Optional separator line
    if (!keyDescriptions.isEmpty()) {
        keyDescriptions.clear();
        text += QString("<tr bgcolor=\"#aaaacc\"><td colspan=2>%1</td></tr>\n")
                    .arg(tr("Standard viewer keys", "In help window keys tab"));
    }

    // 3 - KeyboardAction bindings description
    for (auto it = keyboardBinding_.begin(), end = keyboardBinding_.end();
         it != end; ++it) {
        if ((it.value() != 0) &&
            ((!cameraIsInRotateMode()) || ((it.key() != IncreaseFlySpeed) &&
                                           (it.key() != DecreaseFlySpeed))))
            keyDescriptions[it.value()] = keyboardActionDescription_[it.key()];
    }

    // Add to text in sorted order
    for (auto it = keyDescriptions.begin(), end = keyDescriptions.end();
         it != end; ++it) {
        text += tableLine(keyString(it.key()), it.value());
    }

    // 4 - Camera paths keys description
    const QString cpks = cameraPathKeysString();
    if (!cpks.isNull()) {
        text += "<tr bgcolor=\"#ccccff\"><td colspan=2>\n";
        text += tr("Camera paths are controlled using the %1 keys "
                   "(noted <i>Fx</i> below):",
                   "Help window key tab camera keys")
                    .arg(cpks) +
                "</td></tr>\n";
        text +=
            tableLine(keyString(playPathKeyboardModifiers()) + "<i>" +
                          tr("Fx", "Generic function key (F1..F12)") + "</i>",
                      tr("Plays path (or resets saved position)"));
        text +=
            tableLine(keyString(addKeyFrameKeyboardModifiers()) + "<i>" +
                          tr("Fx", "Generic function key (F1..F12)") + "</i>",
                      tr("Adds a key frame to path (or defines a position)"));
        text += tableLine(
            keyString(addKeyFrameKeyboardModifiers()) + "<i>" +
                tr("Fx", "Generic function key (F1..F12)") + "</i>+<i>" +
                tr("Fx", "Generic function key (F1..F12)") + "</i>",
            tr("Deletes path (or saved position)"));
    }
    text += "</table></center></font>";

    return text;
}

void QtOpenGLViewer::help()
{
    emit helpRequired();

    // TODO: The initialization is a bit hardcoded.
    bool resize = false;
    int width = 600;
    int height = 400;

    static QString label[] = {tr("&Help", "Help window tab title"),
                              tr("&Keyboard", "Help window tab title"),
                              tr("&Mouse", "Help window tab title")};

    if (!helpWidget()) {
        helpWidget_ = new QTabWidget();
        helpWidget()->setWindowTitle(tr("Help", "Help window title"));

        resize = true;
        for (int i = 0; i < 3; ++i) {
            auto *tab = new QTextEdit(nullptr);
            tab->setReadOnly(true);
            helpWidget()->insertTab(i, tab, label[i]);
        }
    }

    for (int i = 0; i < 3; ++i) {
        QString text;
        switch (i) {
            case 0:
                text = helpString();
                break;
            case 1:
                text = keyboardString();
                break;
            case 2:
                text = mouseString();
                break;
            default:
                break;
        }

        auto *textEdit = static_cast<QTextEdit *>(helpWidget()->widget(i));
        textEdit->setHtml(text);
        textEdit->setText(text);

        if (resize && (textEdit->height() > height)) {
            height = textEdit->height();
        }
    }

    if (resize) {
        constexpr int kTabHeight = 40;
        helpWidget()->resize(width, height + kTabHeight);
    }

    helpWidget()->show();
    helpWidget()->raise();
}

QTabWidget *QtOpenGLViewer::helpWidget() { return helpWidget_; }

void QtOpenGLViewer::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == 0) {
        e->ignore();
        return;
    }

    auto it = keyboardBinding_.begin(), end = keyboardBinding_.end();
    const int target = e->keyCombination().toCombined();
    while ((it != end) && (it.value() != target)) {
        ++it;
    }

    const auto key = Qt::Key(e->key());
    const auto modifiers = e->modifiers();

    if (it != end) {
        handleKeyboardAction(it.key());
    }
    else if (pathIndex_.contains(key)) {
        // Camera paths
        unsigned int index = pathIndex_[key];

        // not safe, but try to double press on two viewers at the same time !
        static QElapsedTimer doublePress;

        if (modifiers == playPathKeyboardModifiers()) {
            int elapsed = doublePress.restart();
            if ((elapsed < 250) && (index == previousPathId_))
                camera()->resetPath(index);
            else {
                // Stop previous interpolation before starting a new one.
                if (index != previousPathId_) {
                    KeyFrameInterpolator *previous =
                        camera()->keyFrameInterpolator(previousPathId_);
                    if ((previous) && (previous->interpolationIsStarted()))
                        previous->resetInterpolation();
                }
                camera()->playPath(index);
            }
            previousPathId_ = index;
        }
        else if (modifiers == addKeyFrameKeyboardModifiers()) {
            int elapsed = doublePress.restart();
            if ((elapsed < 250) && (index == previousPathId_)) {
                if (camera()->keyFrameInterpolator(index)) {
                    disconnect(camera()->keyFrameInterpolator(index),
                               &KeyFrameInterpolator::interpolated, this,
                               qOverload<>(&QtOpenGLViewer::update));
                    if (camera()
                            ->keyFrameInterpolator(index)
                            ->numberOfKeyFrames() > 1) {
                        displayMessage(tr("Path %1 deleted", "Feedback message")
                                           .arg(index));
                    }
                    else {
                        displayMessage(
                            tr("Position %1 deleted", "Feedback message")
                                .arg(index));
                    }
                    camera()->deletePath(index);
                }
            }
            else {
                bool nullBefore = !camera()->keyFrameInterpolator(index);
                camera()->addKeyFrameToPath(index);
                if (nullBefore)
                    connect(camera()->keyFrameInterpolator(index),
                            &KeyFrameInterpolator::interpolated, this,
                            qOverload<>(&QtOpenGLViewer::update));
                int nbKF =
                    camera()->keyFrameInterpolator(index)->numberOfKeyFrames();
                if (nbKF > 1)
                    displayMessage(
                        tr("Path %1, position %2 added", "Feedback message")
                            .arg(QString::number(index),
                                 QString::number(nbKF)));
                else
                    displayMessage(
                        tr("Position %1 saved", "Feedback message").arg(index));
            }
            previousPathId_ = index;
        }
        update();
    }
    else {
        if (isValidShortcutKey(key))
            currentlyPressedKey_ = key;
        e->ignore();
    }
}

void QtOpenGLViewer::keyReleaseEvent(QKeyEvent *e)
{
    if (isValidShortcutKey(e->key()))
        currentlyPressedKey_ = Qt::Key(0);
}

void QtOpenGLViewer::handleKeyboardAction(KeyboardAction id)
{
    switch (id) {
        case ShowWorldFrame:
            toggleAxisIsDrawn();
            break;
        case ShowWorldGrid:
            toggleGridIsDrawn();
            break;
        case ShowFPS:
            toggleFPSIsDisplayed();
            break;
        case EnableText:
            toggleTextIsEnabled();
            break;
        case ExitViewer:
            saveStateToFileForAllViewers();
            qApp->closeAllWindows();
            break;
        case SaveScreenshot:
            saveSnapshot(false, false);
            break;
        case FullScreen:
            toggleFullScreen();
            break;
        case ToggleStereoMode:
            toggleStereoDisplay();
            break;
        case ToggleAnimation:
            toggleAnimation();
            break;
        case Help:
            help();
            break;
        case EditCamera:
            toggleCameraIsEdited();
            break;
        case SnapshotToClipboard:
            snapshotToClipboard();
            break;
        case CameraMode:
            toggleCameraMode();
            displayMessage(
                cameraIsInRotateMode()
                    ? tr("Camera in observer mode", "Feedback message")
                    : tr("Camera in fly mode", "Feedback message"));
            break;
        case MoveCameraLeft:
            camera()->frame()->translate(camera()->frame()->inverseTransformOf(
                Vec(-10.0 * camera()->flySpeed(), 0.0, 0.0)));
            update();
            break;
        case MoveCameraRight:
            camera()->frame()->translate(camera()->frame()->inverseTransformOf(
                Vec(10.0 * camera()->flySpeed(), 0.0, 0.0)));
            update();
            break;
        case MoveCameraUp:
            camera()->frame()->translate(camera()->frame()->inverseTransformOf(
                Vec(0.0, 10.0 * camera()->flySpeed(), 0.0)));
            update();
            break;
        case MoveCameraDown:
            camera()->frame()->translate(camera()->frame()->inverseTransformOf(
                Vec(0.0, -10.0 * camera()->flySpeed(), 0.0)));
            update();
            break;

        case IncreaseFlySpeed:
            camera()->setFlySpeed(camera()->flySpeed() * 1.5);
            break;
        case DecreaseFlySpeed:
            camera()->setFlySpeed(camera()->flySpeed() / 1.5);
            break;
        default:
            break;
    }
}

void QtOpenGLViewer::resizeGL(int width, int height)
{
    QOpenGLWidget::resizeGL(width, height);
    glViewport(0, 0, GLint(width), GLint(height));
    camera()->setScreenWidthAndHeight(this->width(), this->height());
}

int QtOpenGLViewer::shortcut(KeyboardAction action) const
{
    if (keyboardBinding_.contains(action))
        return keyboardBinding_[action];

    return 0;
}

void QtOpenGLViewer::setShortcut(KeyboardAction action,
                                 const QKeyCombination &comb)
{
    keyboardBinding_[action] = comb.toCombined();
}

void QtOpenGLViewer::clearShortcuts()
{
    keyboardBinding_.clear();
    pathIndex_.clear();
}

Qt::Key QtOpenGLViewer::pathKey(unsigned int index) const
{
    for (auto it = pathIndex_.begin(), end = pathIndex_.end(); it != end; ++it)
        if (it.value() == index)
            return it.key();
    return Qt::Key(0);
}

void QtOpenGLViewer::setPathKey(int key, unsigned int index)
{
    Qt::Key k = Qt::Key(std::abs(key));
    if (key < 0)
        pathIndex_.remove(k);
    else
        pathIndex_[k] = index;
}

void QtOpenGLViewer::setPlayPathKeyboardModifiers(
    Qt::KeyboardModifiers modifiers)
{
    playPathKeyboardModifiers_ = modifiers;
}

void QtOpenGLViewer::setAddKeyFrameKeyboardModifiers(
    Qt::KeyboardModifiers modifiers)
{
    addKeyFrameKeyboardModifiers_ = modifiers;
}

Qt::KeyboardModifiers QtOpenGLViewer::addKeyFrameKeyboardModifiers() const
{
    return addKeyFrameKeyboardModifiers_;
}

Qt::KeyboardModifiers QtOpenGLViewer::playPathKeyboardModifiers() const
{
    return playPathKeyboardModifiers_;
}

void QtOpenGLViewer::setMouseBinding(Qt::Key key,
                                     Qt::KeyboardModifiers modifiers,
                                     Qt::MouseButton button,
                                     MouseHandler handler, MouseAction action,
                                     bool withConstraint)
{
    if ((handler == FRAME) &&
        ((action == MoveForward) || (action == MoveBackward) ||
         (action == Roll) || (action == LookForward) ||
         (action == ZoomOnRegion))) {
        qWarning("Cannot bind %s to FRAME",
                 mouseActionString(action).toLatin1().constData());
        return;
    }

    if (button == Qt::NoButton) {
        qWarning("No mouse button specified in setMouseBinding");
        return;
    }

    MouseActionPrivate map;
    map.handler = handler;
    map.action = action;
    map.withConstraint = withConstraint;

    MouseBindingPrivate mbp(modifiers, button, key);
    if (action == NoMouseAction)
        mouseBinding_.remove(mbp);
    else
        mouseBinding_.insert(mbp, map);

    ClickBindingPrivate cbp(modifiers, button, false, Qt::NoButton, key);
    clickBinding_.remove(cbp);
}

void QtOpenGLViewer::setMouseBinding(Qt::Key key,
                                     Qt::KeyboardModifiers modifiers,
                                     Qt::MouseButton button, ClickAction action,
                                     bool doubleClick,
                                     Qt::MouseButtons buttonsBefore)
{
    if ((buttonsBefore != Qt::NoButton) && !doubleClick) {
        qWarning()
            << "Buttons before is only meaningful when doubleClick is true in "
               "setMouseBinding().";
        return;
    }

    if (button == Qt::NoButton) {
        qWarning() << "No mouse button specified in setMouseBinding";
        return;
    }

    ClickBindingPrivate cbp(modifiers, button, doubleClick, buttonsBefore, key);
    if (action == NoClick)
        clickBinding_.remove(cbp);
    else
        clickBinding_.insert(cbp, action);

    if (!doubleClick && (buttonsBefore == Qt::NoButton)) {
        MouseBindingPrivate mbp(modifiers, button, key);
        mouseBinding_.remove(mbp);
    }
}

void QtOpenGLViewer::setWheelBinding(Qt::Key key,
                                     Qt::KeyboardModifiers modifiers,
                                     MouseHandler handler, MouseAction action,
                                     bool withConstraint)
{
    if ((action != Zoom) && (action != MoveForward) &&
        (action != MoveBackward) && (action != NoMouseAction)) {
        qWarning("Cannot bind %s to wheel",
                 mouseActionString(action).toLatin1().constData());
        return;
    }

    if ((handler == FRAME) && (action != Zoom) && (action != NoMouseAction)) {
        qWarning("Cannot bind %s to FRAME wheel",
                 mouseActionString(action).toLatin1().constData());
        return;
    }

    MouseActionPrivate map;
    map.handler = handler;
    map.action = action;
    map.withConstraint = withConstraint;

    WheelBindingPrivate wbp(modifiers, key);
    if (action == NoMouseAction)
        wheelBinding_.remove(wbp);
    else
        wheelBinding_[wbp] = map;
}

void QtOpenGLViewer::clearMouseBindings()
{
    mouseBinding_.clear();
    clickBinding_.clear();
    wheelBinding_.clear();
}

QtOpenGLViewer::MouseAction QtOpenGLViewer::mouseAction(
    Qt::Key key, Qt::KeyboardModifiers modifiers, Qt::MouseButton button) const
{
    MouseBindingPrivate mbp(modifiers, button, key);
    if (mouseBinding_.contains(mbp))
        return mouseBinding_[mbp].action;

    return NoMouseAction;
}

int QtOpenGLViewer::mouseHandler(Qt::Key key, Qt::KeyboardModifiers modifiers,
                                 Qt::MouseButton button) const
{
    MouseBindingPrivate mbp(modifiers, button, key);
    if (mouseBinding_.contains(mbp))
        return mouseBinding_[mbp].handler;

    return -1;
}

void QtOpenGLViewer::getWheelActionBinding(
    MouseHandler handler, MouseAction action, bool withConstraint, Qt::Key &key,
    Qt::KeyboardModifiers &modifiers) const
{
    const MouseActionPrivate input{handler, action, withConstraint};
    for (auto it = wheelBinding_.begin(), end = wheelBinding_.end(); it != end;
         ++it)
        if (it.value() == input) {
            key = it.key().key;
            modifiers = it.key().modifiers;
            return;
        }

    key = Qt::Key(-1);
    modifiers = Qt::NoModifier;
}

void QtOpenGLViewer::getMouseActionBinding(MouseHandler handler,
                                           MouseAction action,
                                           bool withConstraint, Qt::Key &key,
                                           Qt::KeyboardModifiers &modifiers,
                                           Qt::MouseButton &button) const
{
    const MouseActionPrivate input{handler, action, withConstraint};
    for (auto it = mouseBinding_.begin(), end = mouseBinding_.end(); it != end;
         ++it) {
        if (it.value() == input) {
            key = it.key().key;
            modifiers = it.key().modifiers;
            button = it.key().button;
            return;
        }
    }

    key = Qt::Key(0);
    modifiers = Qt::NoModifier;
    button = Qt::NoButton;
}

QtOpenGLViewer::MouseAction QtOpenGLViewer::wheelAction(
    Qt::Key key, Qt::KeyboardModifiers modifiers) const
{
    WheelBindingPrivate wbp(modifiers, key);
    if (wheelBinding_.contains(wbp))
        return wheelBinding_[wbp].action;

    return NoMouseAction;
}

int QtOpenGLViewer::wheelHandler(Qt::Key key,
                                 Qt::KeyboardModifiers modifiers) const
{
    WheelBindingPrivate wbp(modifiers, key);
    if (wheelBinding_.contains(wbp))
        return wheelBinding_[wbp].handler;

    return -1;
}

QtOpenGLViewer::ClickAction QtOpenGLViewer::clickAction(
    Qt::Key key, Qt::KeyboardModifiers modifiers, Qt::MouseButton button,
    bool doubleClick, Qt::MouseButtons buttonsBefore) const
{
    ClickBindingPrivate cbp(modifiers, button, doubleClick, buttonsBefore, key);
    if (clickBinding_.contains(cbp))
        return clickBinding_[cbp];

    return NoClick;
}

void QtOpenGLViewer::getClickActionBinding(
    ClickAction action, Qt::Key &key, Qt::KeyboardModifiers &modifiers,
    Qt::MouseButton &button, bool &doubleClick,
    Qt::MouseButtons &buttonsBefore) const
{
    for (auto it = clickBinding_.begin(), end = clickBinding_.end(); it != end;
         ++it) {
        if (it.value() == action) {
            modifiers = it.key().modifiers;
            button = it.key().button;
            doubleClick = it.key().doubleClick;
            buttonsBefore = it.key().buttonsBefore;
            key = it.key().key;
            return;
        }
    }

    modifiers = Qt::NoModifier;
    button = Qt::NoButton;
    doubleClick = false;
    buttonsBefore = Qt::NoButton;
    key = Qt::Key(0);
}

bool QtOpenGLViewer::cameraIsInRotateMode() const
{
    // #CONNECTION# used in toggleCameraMode() and keyboardString()
    Qt::Key key;
    Qt::KeyboardModifiers modifiers;
    Qt::MouseButton button;
    getMouseActionBinding(CAMERA, Rotate, true /*constraint*/, key, modifiers,
                          button);
    return button != Qt::NoButton;
}

void QtOpenGLViewer::toggleCameraMode()
{
    Qt::Key key;
    Qt::KeyboardModifiers modifiers;
    Qt::MouseButton button;
    getMouseActionBinding(CAMERA, Rotate, true /*constraint*/, key, modifiers,
                          button);
    bool rotateMode = button != Qt::NoButton;

    if (!rotateMode) {
        getMouseActionBinding(CAMERA, MoveForward, true /*constraint*/, key,
                              modifiers, button);
    }

    // #CONNECTION# setDefaultMouseBindings()
    if (rotateMode) {
        camera()->frame()->updateSceneUpVector();
        camera()->frame()->stopSpinning();

        setMouseBinding(modifiers, Qt::LeftButton, CAMERA, MoveForward);
        setMouseBinding(modifiers, Qt::MiddleButton, CAMERA, LookForward);
        setMouseBinding(modifiers, Qt::RightButton, CAMERA, MoveBackward);

        setMouseBinding(Qt::Key_R, modifiers, Qt::LeftButton, CAMERA, Roll);

        setMouseBinding(Qt::NoModifier, Qt::LeftButton, NoClick, true);
        setMouseBinding(Qt::NoModifier, Qt::MiddleButton, NoClick, true);
        setMouseBinding(Qt::NoModifier, Qt::RightButton, NoClick, true);

        setWheelBinding(modifiers, CAMERA, MoveForward);
    }
    else {
        // Should stop flyTimer. But unlikely and not easy.
        setMouseBinding(modifiers, Qt::LeftButton, CAMERA, Rotate);
        setMouseBinding(modifiers, Qt::MiddleButton, CAMERA, Zoom);
        setMouseBinding(modifiers, Qt::RightButton, CAMERA, Translate);

        setMouseBinding(Qt::Key_R, modifiers, Qt::LeftButton, CAMERA,
                        ScreenRotate);

        setMouseBinding(Qt::NoModifier, Qt::LeftButton, AlignCamera, true);
        setMouseBinding(Qt::NoModifier, Qt::MiddleButton, ShowEntireScene,
                        true);
        setMouseBinding(Qt::NoModifier, Qt::RightButton, CenterScene, true);

        setWheelBinding(modifiers, CAMERA, Zoom);
    }
}

viewer::ManipulatedFrame *QtOpenGLViewer::manipulatedFrame() const
{
    return manipulatedFrame_;
}

void QtOpenGLViewer::setManipulatedFrame(ManipulatedFrame *frame)
{
    if (manipulatedFrame()) {
        manipulatedFrame()->stopSpinning();

        if (manipulatedFrame() != camera()->frame()) {
            disconnect(manipulatedFrame(), &ManipulatedFrame::manipulated, this,
                       qOverload<>(&QtOpenGLViewer::update));
            disconnect(manipulatedFrame(), &ManipulatedFrame::spun, this,
                       qOverload<>(&QtOpenGLViewer::update));
        }
    }

    manipulatedFrame_ = frame;

    manipulatedFrameIsACamera_ =
        ((manipulatedFrame() != camera()->frame()) &&
         (dynamic_cast<ManipulatedCameraFrame *>(manipulatedFrame())));

    if (manipulatedFrame()) {
        // Prevent multiple connections, that would result in useless display
        // updates
        if (manipulatedFrame() != camera()->frame()) {
            connect(manipulatedFrame(), &ManipulatedFrame::manipulated, this,
                    qOverload<>(&QtOpenGLViewer::update));
            connect(manipulatedFrame(), &ManipulatedFrame::spun, this,
                    qOverload<>(&QtOpenGLViewer::update));
        }
    }
}

void QtOpenGLViewer::drawVisualHints()
{
    // Pivot point cross
    if (visualHint_ & 1) {
        const qreal size = 15.0;
        Vec proj = camera()->projectedCoordinatesOf(camera()->pivotPoint());
        startScreenCoordinatesSystem();
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glLineWidth(3.0);
        glBegin(GL_LINES);
        glVertex2d(proj.x - size, proj.y);
        glVertex2d(proj.x + size, proj.y);
        glVertex2d(proj.x, proj.y - size);
        glVertex2d(proj.x, proj.y + size);
        glEnd();
        glEnable(GL_DEPTH_TEST);
        stopScreenCoordinatesSystem();
    }

    // if (visualHint_ & 2)
    // drawText(80, 10, "Play");

    // Screen rotate line
    ManipulatedFrame *mf = nullptr;
    Vec pnt;
    if (camera()->frame()->action_ == ScreenRotate) {
        mf = camera()->frame();
        pnt = camera()->pivotPoint();
    }
    if (manipulatedFrame() && (manipulatedFrame()->action_ == ScreenRotate)) {
        mf = manipulatedFrame();
        // Maybe useful if the mf is a manipCameraFrame...
        // pnt = manipulatedFrame()->pivotPoint();
        pnt = manipulatedFrame()->position();
    }

    if (mf) {
        pnt = camera()->projectedCoordinatesOf(pnt);
        startScreenCoordinatesSystem();
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glLineWidth(3.0);
        glBegin(GL_LINES);
        glVertex2d(pnt.x, pnt.y);
        glVertex2i(mf->prevPos_.x(), mf->prevPos_.y());
        glEnd();
        glEnable(GL_DEPTH_TEST);
        stopScreenCoordinatesSystem();
    }

    // Zoom on region: draw a rectangle
    if (camera()->frame()->action_ == ZoomOnRegion) {
        startScreenCoordinatesSystem();
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glLineWidth(2.0);
        glBegin(GL_LINE_LOOP);
        glVertex2i(camera()->frame()->pressPos_.x(),
                   camera()->frame()->pressPos_.y());
        glVertex2i(camera()->frame()->prevPos_.x(),
                   camera()->frame()->pressPos_.y());
        glVertex2i(camera()->frame()->prevPos_.x(),
                   camera()->frame()->prevPos_.y());
        glVertex2i(camera()->frame()->pressPos_.x(),
                   camera()->frame()->prevPos_.y());
        glEnd();
        glEnable(GL_DEPTH_TEST);
        stopScreenCoordinatesSystem();
    }
}

void QtOpenGLViewer::setVisualHintsMask(int mask, int delay)
{
    visualHint_ = visualHint_ | mask;
    QTimer::singleShot(delay, this, &QtOpenGLViewer::resetVisualHints);
}

void QtOpenGLViewer::resetVisualHints() { visualHint_ = 0; }

void QtOpenGLViewer::drawArrow(qreal length, qreal radius, int nbSubdivisions)
{
    static GLUquadric *quadric = gluNewQuadric();

    if (radius < 0.0) {
        radius = 0.05 * length;
    }

    const qreal head = 2.5 * (radius / length) + 0.1;
    const qreal coneRadiusCoef = 4.0 - 5.0 * head;

    gluCylinder(quadric, radius, radius, length * (1.0 - head / coneRadiusCoef),
                nbSubdivisions, 1);
    glTranslated(0.0, 0.0, length * (1.0 - head));
    gluCylinder(quadric, coneRadiusCoef * radius, 0.0, head * length,
                nbSubdivisions, 1);
    glTranslated(0.0, 0.0, -length * (1.0 - head));
}

void QtOpenGLViewer::drawArrow(const Vec &from, const Vec &to, qreal radius,
                               int nbSubdivisions)
{
    glPushMatrix();
    glTranslated(from[0], from[1], from[2]);
    const Vec dir = to - from;
    glMultMatrixd(Quaternion(Vec(0, 0, 1), dir).matrix());
    QtOpenGLViewer::drawArrow(dir.norm(), radius, nbSubdivisions);
    glPopMatrix();
}

void QtOpenGLViewer::drawAxis(qreal length)
{
    const qreal charWidth = length / 40.0;
    const qreal charHeight = length / 30.0;
    const qreal charShift = 1.04 * length;

    GLboolean lighting, colorMaterial;
    glGetBooleanv(GL_LIGHTING, &lighting);
    glGetBooleanv(GL_COLOR_MATERIAL, &colorMaterial);

    glDisable(GL_LIGHTING);

    glBegin(GL_LINES);
    // The X
    glVertex3d(charShift, charWidth, -charHeight);
    glVertex3d(charShift, -charWidth, charHeight);
    glVertex3d(charShift, -charWidth, -charHeight);
    glVertex3d(charShift, charWidth, charHeight);
    // The Y
    glVertex3d(charWidth, charShift, charHeight);
    glVertex3d(0.0, charShift, 0.0);
    glVertex3d(-charWidth, charShift, charHeight);
    glVertex3d(0.0, charShift, 0.0);
    glVertex3d(0.0, charShift, 0.0);
    glVertex3d(0.0, charShift, -charHeight);
    // The Z
    glVertex3d(-charWidth, charHeight, charShift);
    glVertex3d(charWidth, charHeight, charShift);
    glVertex3d(charWidth, charHeight, charShift);
    glVertex3d(-charWidth, -charHeight, charShift);
    glVertex3d(-charWidth, -charHeight, charShift);
    glVertex3d(charWidth, -charHeight, charShift);
    glEnd();

    glEnable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);

    float color[4];
    color[0] = 0.7f;
    color[1] = 0.7f;
    color[2] = 1.0f;
    color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    QtOpenGLViewer::drawArrow(length, 0.01 * length);

    color[0] = 1.0f;
    color[1] = 0.7f;
    color[2] = 0.7f;
    color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    glPushMatrix();
    glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
    QtOpenGLViewer::drawArrow(length, 0.01 * length);
    glPopMatrix();

    color[0] = 0.7f;
    color[1] = 1.0f;
    color[2] = 0.7f;
    color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    glPushMatrix();
    glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
    QtOpenGLViewer::drawArrow(length, 0.01 * length);
    glPopMatrix();

    if (colorMaterial) {
        glEnable(GL_COLOR_MATERIAL);
    }
    if (!lighting) {
        glDisable(GL_LIGHTING);
    }
}

void QtOpenGLViewer::drawGrid(qreal size, int nbSubdivisions)
{
    GLboolean lighting;
    glGetBooleanv(GL_LIGHTING, &lighting);

    glDisable(GL_LIGHTING);

    glBegin(GL_LINES);
    for (int i = 0; i <= nbSubdivisions; ++i) {
        const qreal pos = size * (2.0 * i / nbSubdivisions - 1.0);
        glVertex2d(pos, -size);
        glVertex2d(pos, +size);
        glVertex2d(-size, pos);
        glVertex2d(size, pos);
    }
    glEnd();

    if (lighting) {
        glEnable(GL_LIGHTING);
    }
}

QString QtOpenGLViewer::stateFileName() const
{
    QString name = stateFileName_;

    if (!name.isEmpty() && QtOpenGLViewer::QGLViewerIndex(this) > 0) {
        QFileInfo fi(name);
        if (fi.suffix().isEmpty()) {
            name += QString::number(QtOpenGLViewer::QGLViewerIndex(this));
        }
        else {
            name = fi.absolutePath() + '/' + fi.completeBaseName() +
                   QString::number(QtOpenGLViewer::QGLViewerIndex(this)) + "." +
                   fi.suffix();
        }
    }

    return name;
}

void QtOpenGLViewer::setStateFileName(const QString &name)
{
    stateFileName_ = name;
}

void QtOpenGLViewer::saveStateToFile()
{
    QString name = stateFileName();
    if (name.isEmpty()) {
        return;
    }

    QFileInfo fileInfo(name);
    if (fileInfo.isDir()) {
        QMessageBox::warning(
            this, tr("Save to file error", "Message box window title"),
            tr("State file name (%1) references a directory instead of a file.")
                .arg(name));
        return;
    }

    const QString dirName = fileInfo.absolutePath();
    if (!QFileInfo::exists(dirName)) {
        QDir dir;
        if (!dir.mkdir(dirName)) {
            QMessageBox::warning(
                this, tr("Save to file error", "Message box window title"),
                tr("Unable to create directory %1").arg(dirName));
            return;
        }
    }

    // Write the DOM tree to file
    QFile f(name);
    if (!f.open(QIODevice::WriteOnly)) {
        QMessageBox::warning(
            this, tr("Save to file error", "Message box window title"),
            tr("Unable to save to file %1").arg(name) + ":\n" +
                f.errorString());
        return;
    }

    QTextStream out(&f);
    QDomDocument doc("QGLVIEWER");
    doc.appendChild(domElement("QGLViewer", doc));
    doc.save(out, 2);
    f.flush();
    f.close();
}

bool QtOpenGLViewer::restoreStateFromFile()
{
    QString name = stateFileName();
    if (name.isEmpty()) {
        return false;
    }

    QFileInfo fileInfo(name);
    if (!fileInfo.isFile()) {
        // No warning since it would be displayed at first start.
        return false;
    }

    if (!fileInfo.isReadable()) {
        QMessageBox::warning(
            this,
            tr("Problem in state restoration", "Message box window title"),
            tr("File %1 is not readable.").arg(name));
        return false;
    }

    // Read the DOM tree form file
    QFile f(name);
    if (!f.open(QIODevice::ReadOnly)) {
        QMessageBox::warning(
            this, tr("Open file error", "Message box window title"),
            tr("Unable to open file %1").arg(name) + ":\n" + f.errorString());
        return false;
    }

    QDomDocument doc;
    doc.setContent(&f);
    f.close();
    QDomElement main = doc.documentElement();
    initFromDOMElement(main);
    return true;
}

namespace key {
constexpr char kStateTag[]{"State"};
constexpr char kForegroundColor[]{"foregroundColor"};
constexpr char kBackgroundColor[]{"backgroundColor"};
constexpr char kDisplayInStereo[]{"stereo"};
constexpr char kDisplayTag[]{"Display"};
constexpr char kShowAxis[]{"axisIsDrawn"};
constexpr char kShowGrid[]{"gridIsDrawn"};
constexpr char kShowFps[]{"FPSIsDisplayed"};
constexpr char kGeometryTag[]{"Geometry"};
constexpr char kFullScreen[]{"fullScreen"};
} // namespace key

QDomElement QtOpenGLViewer::domElement(const QString &name,
                                       QDomDocument &doc) const
{
    auto de = doc.createElement(name);
    de.setAttribute("version", QGLViewerVersionString());

    auto stateNode = doc.createElement(key::kStateTag);
    // hasMouseTracking() is not saved
    stateNode.appendChild(
        io::QColorDomElement(foregroundColor(), key::kForegroundColor, doc));
    stateNode.appendChild(
        io::QColorDomElement(backgroundColor(), key::kBackgroundColor, doc));
    io::setBoolAttribute(stateNode, key::kDisplayInStereo, displaysInStereo());
    // Revolve or fly camera mode is not saved
    de.appendChild(stateNode);

    auto displayNode = doc.createElement(key::kDisplayTag);
    io::setBoolAttribute(displayNode, key::kShowAxis, axisIsDrawn());
    io::setBoolAttribute(displayNode, key::kShowGrid, gridIsDrawn());
    io::setBoolAttribute(displayNode, key::kShowFps, FPSIsDisplayed());
    io::setBoolAttribute(displayNode, "cameraIsEdited", cameraIsEdited());
    // textIsEnabled() is not saved
    de.appendChild(displayNode);

    auto geometryNode = doc.createElement(key::kGeometryTag);
    io::setBoolAttribute(geometryNode, key::kFullScreen, isFullScreen());
    if (isFullScreen()) {
        geometryNode.setAttribute("prevPosX", QString::number(prevPos_.x()));
        geometryNode.setAttribute("prevPosY", QString::number(prevPos_.y()));
    }
    else {
        QWidget *tlw = topLevelWidget();
        geometryNode.setAttribute("width", QString::number(tlw->width()));
        geometryNode.setAttribute("height", QString::number(tlw->height()));
        geometryNode.setAttribute("posX", QString::number(tlw->pos().x()));
        geometryNode.setAttribute("posY", QString::number(tlw->pos().y()));
    }
    de.appendChild(geometryNode);

    // Restore original Camera zClippingCoefficient before saving.
    if (cameraIsEdited()) {
        camera()->setZClippingCoefficient(previousCameraZClippingCoefficient_);
    }
    de.appendChild(camera()->domElement("Camera", doc));
    if (cameraIsEdited()) {
        camera()->setZClippingCoefficient(5.0);
    }

    if (manipulatedFrame()) {
        de.appendChild(manipulatedFrame()->domElement("ManipulatedFrame", doc));
    }

    return de;
}

void QtOpenGLViewer::initFromDOMElement(const QDomElement &element)
{
    const QString version = element.attribute("version");
    // if (version != QGLViewerVersionString())
    if (version[0] != '2')
        // Patches for previous versions should go here when the state file
        // syntax is modified.
        qWarning("State file created using QGLViewer version %s may not be "
                 "correctly read.",
                 version.toLatin1().constData());

    QDomElement child = element.firstChild().toElement();
    bool tmpCameraIsEdited = cameraIsEdited();
    while (!child.isNull()) {
        if (child.tagName() == key::kStateTag) {
            // #CONNECTION# default values from defaultConstructor()
            // setMouseTracking(DomUtils::boolFromDom(child, "mouseTracking",
            // false));
            setStereoDisplay(
                io::boolFromDom(child, key::kDisplayInStereo, false));
            // if ((child.attribute("cameraMode", "revolve") == "fly") &&
            // (cameraIsInRevolveMode())) 	toggleCameraMode();

            QDomElement ch = child.firstChild().toElement();
            while (!ch.isNull()) {
                if (ch.tagName() == key::kForegroundColor)
                    setForegroundColor(io::QColorFromDom(ch));
                if (ch.tagName() == key::kBackgroundColor)
                    setBackgroundColor(io::QColorFromDom(ch));
                ch = ch.nextSibling().toElement();
            }
        }

        if (child.tagName() == key::kDisplayTag) {
            setAxisIsDrawn(io::boolFromDom(child, key::kShowAxis, false));
            setGridIsDrawn(io::boolFromDom(child, key::kShowGrid, false));
            setFPSIsDisplayed(io::boolFromDom(child, key::kShowFps, false));
            tmpCameraIsEdited = io::boolFromDom(child, "cameraIsEdited", false);
            // setTextIsEnabled(DomUtils::boolFromDom(child, "textIsEnabled",
            // true));
        }

        if (child.tagName() == key::kGeometryTag) {
            setFullScreen(io::boolFromDom(child, key::kFullScreen, false));

            if (isFullScreen()) {
                prevPos_.setX(io::intFromDom(child, "prevPosX", 0));
                prevPos_.setY(io::intFromDom(child, "prevPosY", 0));
            }
            else {
                int width = io::intFromDom(child, "width", 600);
                int height = io::intFromDom(child, "height", 400);
                topLevelWidget()->resize(width, height);
                camera()->setScreenWidthAndHeight(this->width(),
                                                  this->height());

                QPoint pos;
                pos.setX(io::intFromDom(child, "posX", 0));
                pos.setY(io::intFromDom(child, "posY", 0));
                topLevelWidget()->move(pos);
            }
        }

        if (child.tagName() == "Camera") {
            connectAllCameraKFIInterpolatedSignals(false);
            camera()->initFromDOMElement(child);
            connectAllCameraKFIInterpolatedSignals();
        }

        if ((child.tagName() == "ManipulatedFrame") && (manipulatedFrame()))
            manipulatedFrame()->initFromDOMElement(child);

        child = child.nextSibling().toElement();
    }

    // The Camera always stores its "real" zClippingCoef in domElement(). If it
    // is edited, its "real" coef must be saved and the coef set to 5.0, as is
    // done in setCameraIsEdited(). BUT : Camera and Display are read in an
    // arbitrary order. We must initialize Camera's "real" coef BEFORE calling
    // setCameraIsEdited. Hence this temp cameraIsEdited and delayed call
    cameraIsEdited_ = tmpCameraIsEdited;
    if (cameraIsEdited_) {
        previousCameraZClippingCoefficient_ = camera()->zClippingCoefficient();
        // #CONNECTION# 5.0 from setCameraIsEdited.
        camera()->setZClippingCoefficient(5.0);
    }
}

void QtOpenGLViewer::copyBufferToTexture(GLint internalFormat, GLenum format)
{
    int h = 16;
    int w = 16;
    // Todo compare performance with qt code.
    while (w < width()) w <<= 1;
    while (h < height()) h <<= 1;

    bool init = false;
    if ((w != bufferTextureWidth_) || (h != bufferTextureHeight_)) {
        bufferTextureWidth_ = w;
        bufferTextureHeight_ = h;
        bufferTextureMaxU_ = width() / qreal(bufferTextureWidth_);
        bufferTextureMaxV_ = height() / qreal(bufferTextureHeight_);
        init = true;
    }

    if (bufferTextureId() == 0) {
        glGenTextures(1, &bufferTextureId_);
        glBindTexture(GL_TEXTURE_2D, bufferTextureId_);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        init = true;
    }
    else {
        glBindTexture(GL_TEXTURE_2D, bufferTextureId_);
    }

    if ((format != previousBufferTextureFormat_) ||
        (internalFormat != previousBufferTextureInternalFormat_)) {
        previousBufferTextureFormat_ = format;
        previousBufferTextureInternalFormat_ = internalFormat;
        init = true;
    }

    if (init) {
        if (format == GL_NONE) {
            format = GLenum(internalFormat);
        }

        glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, bufferTextureWidth_,
                     bufferTextureHeight_, 0, format, GL_UNSIGNED_BYTE,
                     nullptr);
    }

    glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, width(), height());
}

GLuint QtOpenGLViewer::bufferTextureId() const
{
    if (glIsTexture(bufferTextureId_)) {
        return bufferTextureId_;
    }

    return 0;
}

qreal QtOpenGLViewer::bufferTextureMaxU() const { return bufferTextureMaxU_; }

qreal QtOpenGLViewer::bufferTextureMaxV() const { return bufferTextureMaxV_; }

QFont QtOpenGLViewer::scaledFont(const QFont &font) const
{
    if (!tileRegion_) {
        return font;
    }

    QFont f(font);
    if (f.pixelSize() == -1)
        f.setPointSizeF(f.pointSizeF() * tileRegion_->textScale);
    else
        f.setPixelSize(int(f.pixelSize() * tileRegion_->textScale));
    return f;
}
