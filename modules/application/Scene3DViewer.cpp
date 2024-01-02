#include "Scene3DViewer.h"

#include <GL/glu.h>

#include <QColor>
#include <QDebug>
#include <QKeyEvent>

#include <AxMVS/Scene>
#include <AxScene3D/Camera>
#include <AxScene3D/ManipulatedFrame>

using namespace viewer;

namespace thoht {

namespace {
constexpr int nbKeyFrames{4};

inline Vec vector3ToVec(const Eigen::Vector3d vec3)
{
    return {vec3.x(), vec3.y(), vec3.z()};
}

void drawCameraPos(const Camera &camera, const QColor &color = Qt::green)
{
    glColor3f(color.redF(), color.greenF(), color.blueF());
    glVertex3fv(vector3ToVec(camera.position()));
}

// TODO: Copy from QGLViewer
void drawAxis(qreal length, const QColor &qcolor)
{
    constexpr qreal kRadiusToLength = 0.05;

    GLboolean lighting, colorMaterial;
    glGetBooleanv(GL_LIGHTING, &lighting);
    glGetBooleanv(GL_COLOR_MATERIAL, &colorMaterial);

    glDisable(GL_COLOR_MATERIAL);

    float color[4];
    color[0] = qcolor.redF();
    color[1] = qcolor.greenF();
    color[2] = qcolor.blueF();
    color[3] = qcolor.alphaF();
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    QtOpenGLViewer::drawArrow(length, kRadiusToLength * length);

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    glPushMatrix();
    glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
    QtOpenGLViewer::drawArrow(length, kRadiusToLength * length);
    glPopMatrix();

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    glPushMatrix();
    glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
    QtOpenGLViewer::drawArrow(length, kRadiusToLength * length);
    glPopMatrix();

    if (colorMaterial) {
        glEnable(GL_COLOR_MATERIAL);
    }
    if (!lighting) {
        glDisable(GL_LIGHTING);
    }
}

void drawCameraFrame(const Camera &camera, const QColor &color)
{
    static GLdouble m[4][4];

    const Eigen::Quaterniond q{camera.orientationAsRotationMatrix()};
    const auto t = camera.position();

    const qreal q00 = 2.0 * q.x() * q.x();
    const qreal q11 = 2.0 * q.y() * q.y();
    const qreal q22 = 2.0 * q.z() * q.z();

    const qreal q01 = 2.0 * q.x() * q.y();
    const qreal q02 = 2.0 * q.x() * q.z();
    const qreal q03 = 2.0 * q.x() * q.w();

    const qreal q12 = 2.0 * q.y() * q.z();
    const qreal q13 = 2.0 * q.y() * q.w();

    const qreal q23 = 2.0 * q.z() * q.w();

    m[0][0] = 1.0 - q11 - q22;
    m[1][0] = q01 - q23;
    m[2][0] = q02 + q13;

    m[0][1] = q01 + q23;
    m[1][1] = 1.0 - q22 - q00;
    m[2][1] = q12 - q03;

    m[0][2] = q02 - q13;
    m[1][2] = q12 + q03;
    m[2][2] = 1.0 - q11 - q00;

    m[0][3] = 0.0;
    m[1][3] = 0.0;
    m[2][3] = 0.0;

    m[3][0] = t.x();
    m[3][1] = t.y();
    m[3][2] = t.z();
    m[3][3] = 1.0;

    glPushMatrix();
    glMultMatrixd((const GLdouble *)(m));

    drawAxis(0.03f, color);

    glPopMatrix();
}

void drawCamera(const Camera &camera)
{
    glPushMatrix();

    Eigen::Matrix4d transform = Eigen::Matrix4d::Zero();
    transform.block<3, 3>(0, 0) =
        camera.orientationAsRotationMatrix().transpose();
    transform.col(3).head<3>() = camera.position();
    transform(3, 3) = 1.;

    // Apply world pose transformation.
    glMultMatrixd(reinterpret_cast<GLdouble *>(transform.data()));

    // Draw Cameras.

    // Create the camera wireframe.
    // If intrinsic parameters are not set then use the focal length as a guess.
    double norm_focal_length = 1.;
    const double image_width =
        (camera.imageWidth() == 0) ? camera.focalLength() : camera.imageWidth();
    const double image_height = (camera.imageHeight() == 0)
                                    ? camera.focalLength()
                                    : camera.imageHeight();
    const double norm_width = (image_width / 2.) / camera.focalLength();
    const double norm_height = (image_height / 2.) / camera.focalLength();

    const auto tl =
        norm_focal_length * Eigen::Vector3d(-norm_width, -norm_height, 1.);
    const auto tr =
        norm_focal_length * Eigen::Vector3d(norm_width, -norm_height, 1.);
    const auto br =
        norm_focal_length * Eigen::Vector3d(norm_width, norm_height, 1.);
    const auto bl =
        norm_focal_length * Eigen::Vector3d(-norm_width, norm_height, 1.);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // Base
    glBegin(GL_QUADS);
    glVertex3d(tr.x(), tr.y(), tr.z());
    glVertex3d(tl.x(), tl.y(), tl.z());
    glVertex3d(bl.x(), bl.y(), bl.z());
    glVertex3d(br.x(), br.y(), br.z());
    glEnd();

    // Frustum lines
    glBegin(GL_LINES);
    glVertex3d(0., 0., 0.);
    glVertex3d(tr.x(), tr.y(), tr.z());
    glVertex3d(0., 0., 0.);
    glVertex3d(tl.x(), tl.y(), tl.z());
    glVertex3d(0., 0., 0.);
    glVertex3d(bl.x(), bl.y(), bl.z());
    glVertex3d(0., 0., 0.);
    glVertex3d(br.x(), br.y(), br.z());
    glEnd();

    glPopMatrix();
}

} // namespace

namespace viz {

viz::Landmark::Landmark(qreal x, qreal y, qreal z)
    : color_(Qt::blue), pos_(x, y, z)
{
}

void ::thoht::viz::Landmark::setColor(const QColor &color)
{
    if (color.isValid()) {
        color_ = color;
    }
}

void ::thoht::viz::Landmark::draw()
{
    glColor3f(color_.redF(), color_.greenF(), color_.blueF());
    glVertex3fv(pos_);
}

} // namespace viz

class Spiral
{
public:
    Spiral() {}

    void draw()
    {
        const float nbSteps = 200.0;

        glBegin(GL_QUAD_STRIP);
        for (int i = 0; i < nbSteps; ++i) {
            const float ratio = i / nbSteps;
            const float angle = 21.0 * ratio;
            const float c = cos(angle);
            const float s = sin(angle);
            const float r1 = 1.0 - 0.8f * ratio;
            const float r2 = 0.8f - 0.8f * ratio;
            const float alt = ratio - 0.5f;
            const float nor = 0.5f;
            const float up = sqrt(1.0 - nor * nor);
            glColor3f(1.0 - ratio, 0.2f, ratio);
            glNormal3f(nor * c, up, nor * s);
            glVertex3f(r1 * c, alt, r1 * s);
            glVertex3f(r2 * c, alt + 0.05f, r2 * s);
        }
        glEnd();
    }
};

Scene3DViewer::Scene3DViewer(QWidget *parent)
    : QtOpenGLViewer(parent), m_keyFrameIdx{0}
{
    setMouseTracking(true);

    restoreStateFromFile();

    auto *myFrame = new Frame();
    m_interpolator.setFrame(myFrame);
    m_interpolator.setLoopInterpolation();

    m_spiral = new Spiral;

    connect(&m_interpolator, &viewer::KeyFrameInterpolator::interpolated, this,
            qOverload<>(&QWidget::update));
}

void Scene3DViewer::showOrientationsInLine(const QuaterniondList &quats)
{
    if (!m_keyFrames.empty()) {
        qDeleteAll(m_keyFrames);
        m_keyFrames.clear();
    }

    const auto keyFrameCount = quats.size();
    m_keyFrames.reserve(quats.size());
    for (size_t i{0}; i < keyFrameCount; i++) {
        auto keyFrame = new ManipulatedFrame();
        keyFrame->setPosition(-1.0 + 2.0 * i / (keyFrameCount - 1), 0.0, 0.0);
        keyFrame->setOrientation(quats[i].x(), quats[i].y(), quats[i].z(),
                                 quats[i].w());
        m_keyFrames.push_back(keyFrame);
        m_interpolator.addKeyFrame(*keyFrame);
    }

    setManipulatedFrame(m_keyFrames[m_keyFrameIdx]);
    m_interpolator.startInterpolation();
}

void Scene3DViewer::showScene(Scene *scene)
{
    if (!scene) {
        qWarning() << "Failed to show scene: Invalid scene.";
        return;
    }

    m_scene = scene;
    //    m_scene->normalize();

    // Update landmarks
    landmarks_.clear();
    landmarks_.reserve(m_scene->trackCount());
    for (const auto &trackId : m_scene->trackIds()) {
        const auto *track = m_scene->track(trackId);
        if (!track || !track->estimated()) {
            continue;
        }

        //        QVector3D point_coords =
        //        QVector3D(track->position().hnormalized().x(),
        //                                           track->position().hnormalized().y(),
        //                                           track->position().hnormalized().z());

        //        QColor point_color = QColor(static_cast<int>(track->color()(0,
        //        0)),
        //                                    static_cast<int>(track->color()(1,
        //                                    0)),
        //                                    static_cast<int>(track->color()(2,
        //                                    0)));

        //        WorldPoint world_point;
        //        world_point.coords = point_coords;
        //        world_point.color = point_color;
        //        world_point.trackId = trackId;
        //        world_point.is_found = false;
        //        world_point.is_selected = false;

        const auto pos = track->position().hnormalized();
        landmarks_.emplace_back(pos.x(), pos.y(), pos.z());
    }

    // Update views
    l_cameras_.clear();
    l_cameras_.reserve(m_scene->viewCount());
    for (const auto &viewId : m_scene->viewIds()) {
        const auto *view = m_scene->view(viewId);
        if (!view || !view->estimated()) {
            continue;
        }

        ModifiedCamera camera(view->camera());
        camera.setColor(m_scene->cameraId(viewId) == CameraId{0} ? Qt::red
                                                                 : Qt::green);
        camera.SetViewName(QString::fromStdString(view->name()));
        l_cameras_.push_back(camera);
    }

    draw();
    update();
}

void Scene3DViewer::keyPressEvent(QKeyEvent *e)
{
    switch (e->key()) {
        case Qt::Key_Left:
            m_keyFrameIdx = (m_keyFrameIdx + nbKeyFrames - 1) % nbKeyFrames;
            setManipulatedFrame(m_keyFrames[m_keyFrameIdx]);
            update();
            break;
        case Qt::Key_Right:
            m_keyFrameIdx = (m_keyFrameIdx + 1) % nbKeyFrames;
            setManipulatedFrame(m_keyFrames[m_keyFrameIdx]);
            update();
            break;
        case Qt::Key_Return:
            m_interpolator.toggleInterpolation();
            break;
        case Qt::Key_Plus:
            m_interpolator.setInterpolationSpeed(
                m_interpolator.interpolationSpeed() + 0.25);
            break;
        case Qt::Key_Minus:
            m_interpolator.setInterpolationSpeed(
                m_interpolator.interpolationSpeed() - 0.25);
            break;
        // case Qt::Key_C :
        // kfi_.setClosedPath(!kfi_.closedPath());
        // break;
        default:
            QtOpenGLViewer::keyPressEvent(e);
            break;
    }
}

void Scene3DViewer::init()
{
    //    QGLViewer::init();

    glPointSize(3.0);
}

void Scene3DViewer::draw()
{
    //    m_interpolator.drawPath(5, 10);

    //    for (size_t i{0}; i < m_keyFrames.size(); ++i) {
    //        glPushMatrix();
    //        glMultMatrixd(m_interpolator.keyFrame(i).matrix());

    //        if ((i == m_keyFrameIdx) || (m_keyFrames[i]->grabsMouse()))
    //            drawAxis(0.4f);
    //        else
    //            drawAxis(0.2f);

    //        glPopMatrix();
    //    }

    // Draw landmarks
    glBegin(GL_POINTS);
    for (auto &landmark : landmarks_) {
        landmark.draw();
    }
    glEnd();

    // Draw views
    glBegin(GL_POINTS);
    for (const auto &camera : l_cameras_) {
        drawCameraPos(camera, camera.color());
    }
    glEnd();

    // Draw frames
    for (const auto &camera : l_cameras_) {
        drawCameraFrame(camera, camera.color());
    }
}

} // namespace thoht

#include "moc_Scene3DViewer.cpp"
