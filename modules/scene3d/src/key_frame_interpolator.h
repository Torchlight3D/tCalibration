#pragma once

#include <QObject>
#include <QTimer>

#include "frame.h"

namespace viewer {

class Camera;

// TODO:
// 1. closedPath
// 2. insertKeyFrames, deleteKeyFrame, replaceKeyFrame

class KeyFrameInterpolatorPrivate;
class KeyFrameInterpolator : public QObject
{
    Q_OBJECT

public:
    // TODO: parent???
    explicit KeyFrameInterpolator(Frame *frame = nullptr);
    virtual ~KeyFrameInterpolator();

    Frame *frame() const;
    Q_SLOT void setFrame(Frame *const frame);

    Frame keyFrame(int index) const;
    qreal keyFrameTime(int index) const;
    int numberOfKeyFrames() const;

    qreal duration() const;
    qreal firstTime() const;
    qreal lastTime() const;

    qreal interpolationTime() const;
    Q_SLOT void setInterpolationTime(qreal time);

    qreal interpolationSpeed() const;
    Q_SLOT void setInterpolationSpeed(qreal speed);

    int interpolationPeriod() const;
    Q_SLOT void setInterpolationPeriod(int period);

    bool interpolationIsStarted() const;
    Q_SLOT void startInterpolation(int period = -1);
    Q_SLOT void stopInterpolation();
    Q_SLOT void resetInterpolation();
    Q_SLOT inline void toggleInterpolation()
    {
        if (interpolationIsStarted())
            stopInterpolation();
        else
            startInterpolation();
    }

    bool loopInterpolation() const;
    Q_SLOT void setLoopInterpolation(bool loop = true);

    bool closedPath() const;
    Q_SLOT void setClosedPath(bool closed = true);

signals:
    void interpolated();
    void endReached();

public slots:
    void addKeyFrame(const Frame &frame);
    void addKeyFrame(const Frame &frame, qreal time);
    void addKeyFrame(const Frame *const frame);
    void addKeyFrame(const Frame *const frame, qreal time);

    void deletePath();

    virtual void interpolateAtTime(qreal time);

public:
    // TODO: use enum
    virtual void drawPath(int mask = 1, int nbFrames = 6, qreal scale = 1.0);

public:
    virtual QDomElement domElement(const QString &name,
                                   QDomDocument &document) const;
    virtual void initFromDOMElement(const QDomElement &element);

private slots:
    virtual void update();
    virtual void invalidateValues()
    {
        valuesAreValid_ = false;
        pathIsValid_ = false;
        splineCacheIsValid_ = false;
    }

private:
    // KeyFrameInterpolator(const KeyFrameInterpolator& kfi);
    // KeyFrameInterpolator& operator=(const KeyFrameInterpolator& kfi);

    void updateCurrentKeyFrameForTime(qreal time);
    void updateModifiedFrameValues();
    void updateSplineCache();

private:
    class KeyFrame;

    mutable QList<KeyFrame *> keyFrames_;
    QMutableListIterator<KeyFrame *> *currentFrame_[4];
    QList<Frame> path_;

    Frame *frame_;

    QTimer timer_;
    int period_;
    qreal interpolationTime_;
    qreal interpolationSpeed_;
    bool interpolationStarted_;

    bool closedPath_;
    bool loopInterpolation_;

    bool pathIsValid_;
    bool valuesAreValid_;
    bool currentFrameValid_;
    bool splineCacheIsValid_;
    Vec v1_, v2_;
};

} // namespace viewer
