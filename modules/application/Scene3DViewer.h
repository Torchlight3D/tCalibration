#pragma once

#include <QVector3D>

#include <AxCamera/Camera>
#include <AxMath/EigenTypes>
#include <AxMVS/MvsTypes>
#include <AxScene3D/KeyFrameInterpolator>
#include <AxScene3D/Scene3DViewer>
#include <AxScene3D/Vec>

namespace viewer {
class ManipulatedFrame;
}

namespace thoht {

class Scene;
class Spiral;

struct WorldPoint
{
    QVector3D coords;
    QColor color;
    TrackId trackId;
    bool is_found;
    bool is_selected;
};

class Scene;

class ModifiedCamera : public Camera
{
public:
    explicit ModifiedCamera(const Camera& camera)
        : Camera(camera), highlighted_(false)
    {
    }

    void setColor(const QColor& color);
    QColor color() const;

    void SetHighlighted(bool highlighted) { highlighted_ = highlighted; }
    bool IsHighlighted() const { return highlighted_; }

    void SetViewName(const QString& view_name) { view_name_ = view_name; }
    QString GetViewName() const { return view_name_; }

private:
    bool highlighted_;
    QString view_name_;
    QColor color_;
};

inline void ModifiedCamera::setColor(const QColor& color)
{
    if (color.isValid()) {
        color_ = color;
    }
}

inline QColor ModifiedCamera::color() const { return color_; }

namespace viz {

class Landmark
{
public:
    Landmark(qreal x, qreal y, qreal z);

    void setColor(const QColor& color);

    void draw();

private:
    QColor color_;
    viewer::Vec pos_;
};

class View
{
public:
    View();

    void draw();

private:
};

} // namespace viz

class Scene3DViewer : public QtOpenGLViewer
{
    Q_OBJECT

public:
    explicit Scene3DViewer(QWidget* parent = nullptr);

    void addLandmark();

    void addView();
    void showViews();

    void addTrajectory();

    void showOrientationsInLine(const QuaterniondList& quats);

public slots:
    void showScene(Scene* scene);

protected:
    void init() override;
    void draw() override;

    void keyPressEvent(QKeyEvent* event) override;

private:
    Scene* m_scene{nullptr};
    std::vector<viewer::ManipulatedFrame*> m_keyFrames;
    std::vector<WorldPoint> world_points_;
    std::vector<ModifiedCamera> l_cameras_;
    std::vector<ModifiedCamera> r_cameras_;

    viewer::KeyFrameInterpolator m_interpolator;
    int m_keyFrameIdx;

    Spiral* m_spiral{nullptr};

    std::vector<viz::Landmark> landmarks_;
};

} // namespace thoht
