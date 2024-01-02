#include "key_frame_interpolator.h"

#include <QDebug>

#include "dom_utils.h"
#include "viewer.h" // for QGLViewer::drawAxis and Camera::drawCamera

using namespace viewer;

namespace {
constexpr int kDefaultInterpolationPeriod = 40;
}

///------- KeyFrameInterpolator::KeyFrame starts from here
class KeyFrameInterpolator::KeyFrame
{
public:
    KeyFrame(const Frame &fr, qreal t);
    KeyFrame(const Frame *fr, qreal t);

    Vec position() const { return p_; }
    Quaternion orientation() const { return q_; }
    Vec tgP() const { return tgP_; }
    Quaternion tgQ() const { return tgQ_; }
    qreal time() const { return time_; }
    const Frame *frame() const { return frame_; }
    void updateValuesFromPointer();
    void flipOrientationIfNeeded(const Quaternion &prev);
    void computeTangent(const KeyFrame *const prev, const KeyFrame *const next);

private:
    Vec p_, tgP_;
    Quaternion q_, tgQ_;
    qreal time_;
    const Frame *const frame_;
};

KeyFrameInterpolator::KeyFrame::KeyFrame(const Frame &fr, qreal t)
    : time_(t), frame_(nullptr)
{
    p_ = fr.position();
    q_ = fr.orientation();
}

KeyFrameInterpolator::KeyFrame::KeyFrame(const Frame *fr, qreal t)
    : time_(t), frame_(fr)
{
    updateValuesFromPointer();
}

void KeyFrameInterpolator::KeyFrame::updateValuesFromPointer()
{
    p_ = frame()->position();
    q_ = frame()->orientation();
}

void KeyFrameInterpolator::KeyFrame::computeTangent(const KeyFrame *const prev,
                                                    const KeyFrame *const next)
{
    tgP_ = 0.5 * (next->position() - prev->position());
    tgQ_ =
        Quaternion::squadTangent(prev->orientation(), q_, next->orientation());
}

void KeyFrameInterpolator::KeyFrame::flipOrientationIfNeeded(
    const Quaternion &prev)
{
    if (Quaternion::dot(prev, q_) < 0.0) {
        q_.negate();
    }
}

///------- KeyFrameInterpolator starts from here
KeyFrameInterpolator::KeyFrameInterpolator(Frame *frame)
    : frame_(nullptr),
      period_(kDefaultInterpolationPeriod),
      interpolationTime_(0.0),
      interpolationSpeed_(1.0),
      interpolationStarted_(false),
      closedPath_(false),
      loopInterpolation_(false),
      pathIsValid_(false),
      valuesAreValid_(true),
      currentFrameValid_(false)
{
    setFrame(frame);
    for (int i = 0; i < 4; ++i) {
        currentFrame_[i] = new QMutableListIterator<KeyFrame *>(keyFrames_);
    }

    connect(&timer_, &QTimer::timeout, this, &KeyFrameInterpolator::update);
}

KeyFrameInterpolator::~KeyFrameInterpolator()
{
    deletePath();
    for (int i{0}; i < 4; ++i) {
        delete currentFrame_[i];
    }
}

Frame *KeyFrameInterpolator::frame() const { return frame_; }

void KeyFrameInterpolator::setFrame(Frame *const frame)
{
    if (this->frame()) {
        disconnect(this, &KeyFrameInterpolator::interpolated, this->frame(),
                   &Frame::interpolated);
    }

    frame_ = frame;
    if (this->frame()) {
        connect(this, &KeyFrameInterpolator::interpolated, this->frame(),
                &Frame::interpolated);
    }
}

int KeyFrameInterpolator::numberOfKeyFrames() const
{
    return keyFrames_.count();
}

qreal KeyFrameInterpolator::interpolationTime() const
{
    return interpolationTime_;
}

void KeyFrameInterpolator::setInterpolationTime(qreal time)
{
    interpolationTime_ = time;
}

qreal KeyFrameInterpolator::interpolationSpeed() const
{
    return interpolationSpeed_;
}

void KeyFrameInterpolator::setInterpolationSpeed(qreal speed)
{
    interpolationSpeed_ = speed;
}

int KeyFrameInterpolator::interpolationPeriod() const { return period_; }

void KeyFrameInterpolator::setInterpolationPeriod(int period)
{
    period_ = period;
}

void KeyFrameInterpolator::update()
{
    interpolateAtTime(interpolationTime());

    interpolationTime_ += interpolationSpeed() * interpolationPeriod() / 1000.0;

    if (interpolationTime() > keyFrames_.last()->time()) {
        if (loopInterpolation())
            setInterpolationTime(keyFrames_.first()->time() +
                                 interpolationTime_ -
                                 keyFrames_.last()->time());
        else {
            // Make sure last KeyFrame is reached and displayed
            interpolateAtTime(keyFrames_.last()->time());
            stopInterpolation();
        }

        emit endReached();
    }
    else if (interpolationTime() < keyFrames_.first()->time()) {
        if (loopInterpolation())
            setInterpolationTime(keyFrames_.last()->time() -
                                 keyFrames_.first()->time() +
                                 interpolationTime_);
        else {
            // Make sure first KeyFrame is reached and displayed
            interpolateAtTime(keyFrames_.first()->time());
            stopInterpolation();
        }

        emit endReached();
    }
}

bool KeyFrameInterpolator::interpolationIsStarted() const
{
    return interpolationStarted_;
}

void KeyFrameInterpolator::startInterpolation(int period)
{
    if (period >= 0) {
        setInterpolationPeriod(period);
    }

    if (!keyFrames_.isEmpty()) {
        if ((interpolationSpeed() > 0.0) &&
            (interpolationTime() >= keyFrames_.last()->time()))
            setInterpolationTime(keyFrames_.first()->time());
        if ((interpolationSpeed() < 0.0) &&
            (interpolationTime() <= keyFrames_.first()->time()))
            setInterpolationTime(keyFrames_.last()->time());
        timer_.start(interpolationPeriod());
        interpolationStarted_ = true;
        update();
    }
}

void KeyFrameInterpolator::stopInterpolation()
{
    timer_.stop();
    interpolationStarted_ = false;
}

void KeyFrameInterpolator::resetInterpolation()
{
    stopInterpolation();
    setInterpolationTime(firstTime());
}

bool KeyFrameInterpolator::loopInterpolation() const
{
    return loopInterpolation_;
}

void KeyFrameInterpolator::setLoopInterpolation(bool loop)
{
    loopInterpolation_ = loop;
}

bool KeyFrameInterpolator::closedPath() const { return closedPath_; }

void KeyFrameInterpolator::setClosedPath(bool closed) { closedPath_ = closed; }

void KeyFrameInterpolator::addKeyFrame(const Frame *const frame, qreal time)
{
    if (!frame) {
        return;
    }

    if (keyFrames_.isEmpty()) {
        interpolationTime_ = time;
    }

    if ((!keyFrames_.isEmpty()) && (keyFrames_.last()->time() > time)) {
        qWarning() << "Failed to add key frame: Time is not monotone.";
    }
    else {
        keyFrames_.append(new KeyFrame(frame, time));
    }

    connect(frame, &Frame::modified, this,
            &KeyFrameInterpolator::invalidateValues);

    valuesAreValid_ = false;
    pathIsValid_ = false;
    currentFrameValid_ = false;
    resetInterpolation();
}

void KeyFrameInterpolator::addKeyFrame(const Frame &frame, qreal time)
{
    if (keyFrames_.isEmpty()) {
        interpolationTime_ = time;
    }

    if ((!keyFrames_.isEmpty()) && (keyFrames_.last()->time() > time)) {
        qWarning() << "Error in KeyFrameInterpolator::addKeyFrame: time is not "
                      "monotone";
    }
    else {
        keyFrames_.append(new KeyFrame(frame, time));
    }

    valuesAreValid_ = false;
    pathIsValid_ = false;
    currentFrameValid_ = false;
    resetInterpolation();
}

void KeyFrameInterpolator::addKeyFrame(const Frame *const frame)
{
    qreal time = keyFrames_.isEmpty() ? 0. : (lastTime() + 1.);
    addKeyFrame(frame, time);
}

void KeyFrameInterpolator::addKeyFrame(const Frame &frame)
{
    qreal time = keyFrames_.isEmpty() ? 0. : (keyFrames_.last()->time() + 1.);
    addKeyFrame(frame, time);
}

void KeyFrameInterpolator::deletePath()
{
    stopInterpolation();
    qDeleteAll(keyFrames_);
    keyFrames_.clear();
    pathIsValid_ = false;
    valuesAreValid_ = false;
    currentFrameValid_ = false;
}

static void drawCamera(qreal scale)
{
    glDisable(GL_LIGHTING);

    const qreal halfHeight = scale * 0.07;
    const qreal halfWidth = halfHeight * 1.3;
    const qreal dist = halfHeight / tan(qreal(M_PI) / 8.0);

    const qreal arrowHeight = 1.5 * halfHeight;
    const qreal baseHeight = 1.2 * halfHeight;
    const qreal arrowHalfWidth = 0.5 * halfWidth;
    const qreal baseHalfWidth = 0.3 * halfWidth;

    // Frustum outline
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_LINE_STRIP);
    glVertex3d(-halfWidth, halfHeight, -dist);
    glVertex3d(-halfWidth, -halfHeight, -dist);
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(halfWidth, -halfHeight, -dist);
    glVertex3d(-halfWidth, -halfHeight, -dist);
    glEnd();
    glBegin(GL_LINE_STRIP);
    glVertex3d(halfWidth, -halfHeight, -dist);
    glVertex3d(halfWidth, halfHeight, -dist);
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(-halfWidth, halfHeight, -dist);
    glVertex3d(halfWidth, halfHeight, -dist);
    glEnd();

    // Up arrow
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // Base
    glBegin(GL_QUADS);
    glVertex3d(-baseHalfWidth, halfHeight, -dist);
    glVertex3d(baseHalfWidth, halfHeight, -dist);
    glVertex3d(baseHalfWidth, baseHeight, -dist);
    glVertex3d(-baseHalfWidth, baseHeight, -dist);
    glEnd();

    // Arrow
    glBegin(GL_TRIANGLES);
    glVertex3d(0.0, arrowHeight, -dist);
    glVertex3d(-arrowHalfWidth, baseHeight, -dist);
    glVertex3d(arrowHalfWidth, baseHeight, -dist);
    glEnd();
}

void KeyFrameInterpolator::drawPath(int mask, int nbFrames, qreal scale)
{
    constexpr int nbSteps{30};
    if (!pathIsValid_) {
        path_.clear();

        if (keyFrames_.isEmpty()) {
            return;
        }

        if (!valuesAreValid_) {
            updateModifiedFrameValues();
        }

        if (keyFrames_.count() == 1) {
            path_.emplace_back(keyFrames_.front()->position(),
                               keyFrames_.front()->orientation());
        }
        else {
            static Frame fr;
            KeyFrame *kf_[4];
            kf_[0] = keyFrames_.first();
            kf_[1] = kf_[0];
            int index = 1;
            kf_[2] =
                (index < keyFrames_.size()) ? keyFrames_.at(index) : nullptr;
            index++;
            kf_[3] =
                (index < keyFrames_.size()) ? keyFrames_.at(index) : nullptr;

            while (kf_[2]) {
                Vec diff = kf_[2]->position() - kf_[1]->position();
                Vec v1 = 3.0 * diff - 2.0 * kf_[1]->tgP() - kf_[2]->tgP();
                Vec v2 = -2.0 * diff + kf_[1]->tgP() + kf_[2]->tgP();

                // cout << kf_[0]->time() << " , " << kf_[1]->time() << " , " <<
                // kf_[2]->time() << " , " << kf_[3]->time() << endl;
                for (int step = 0; step < nbSteps; ++step) {
                    qreal alpha = step / static_cast<qreal>(nbSteps);
                    fr.setPosition(
                        kf_[1]->position() +
                        alpha * (kf_[1]->tgP() + alpha * (v1 + alpha * v2)));
                    fr.setOrientation(Quaternion::squad(
                        kf_[1]->orientation(), kf_[1]->tgQ(), kf_[2]->tgQ(),
                        kf_[2]->orientation(), alpha));
                    path_.push_back(fr);
                }

                // Shift
                kf_[0] = kf_[1];
                kf_[1] = kf_[2];
                kf_[2] = kf_[3];
                index++;
                kf_[3] = (index < keyFrames_.size()) ? keyFrames_.at(index)
                                                     : nullptr;
            }
            // Add last KeyFrame
            path_.emplace_back(kf_[1]->position(), kf_[1]->orientation());
        }
        pathIsValid_ = true;
    }

    if (mask) {
        glDisable(GL_LIGHTING);
        glLineWidth(2);

        if (mask & 1) {
            glBegin(GL_LINE_STRIP);
            for (const auto &frame : path_) {
                glVertex3fv(frame.position());
            }
            glEnd();
        }

        if (mask & 6) {
            int count = 0;
            nbFrames = qMin(nbSteps, nbFrames);
            qreal goal = 0.0;
            for (const auto &frame : path_)
                if ((count++) >= goal) {
                    goal += nbSteps / static_cast<qreal>(nbFrames);
                    glPushMatrix();
                    glMultMatrixd(frame.matrix());
                    if (mask & 2) {
                        drawCamera(scale);
                    }
                    if (mask & 4) {
                        QtOpenGLViewer::drawAxis(scale / 10.0);
                    }
                    glPopMatrix();
                }
        }
    }
}

void KeyFrameInterpolator::updateModifiedFrameValues()
{
    auto prevQ = keyFrames_.first()->orientation();
    for (auto *kf : keyFrames_) {
        if (kf->frame()) {
            kf->updateValuesFromPointer();
        }
        kf->flipOrientationIfNeeded(prevQ);
        prevQ = kf->orientation();
    }

    auto *prev = keyFrames_.first();
    auto *curr = keyFrames_.first();
    int index = 1;
    while (curr) {
        index++;

        auto *next =
            (index < keyFrames_.size()) ? keyFrames_.at(index) : nullptr;
        if (next) {
            curr->computeTangent(prev, next);
        }
        else {
            curr->computeTangent(prev, curr);
        }

        prev = curr;
        curr = next;
    }
    valuesAreValid_ = true;
}

Frame KeyFrameInterpolator::keyFrame(int index) const
{
    const KeyFrame *const frame = keyFrames_.at(index);
    return {frame->position(), frame->orientation()};
}

qreal KeyFrameInterpolator::keyFrameTime(int index) const
{
    return keyFrames_.at(index)->time();
}

qreal KeyFrameInterpolator::duration() const
{
    return lastTime() - firstTime();
}

qreal KeyFrameInterpolator::firstTime() const
{
    return keyFrames_.empty() ? 0. : keyFrames_.first()->time();
}

qreal KeyFrameInterpolator::lastTime() const
{
    return keyFrames_.empty() ? 0. : keyFrames_.last()->time();
}

void KeyFrameInterpolator::updateCurrentKeyFrameForTime(qreal time)
{
    // Assertion: times are sorted in monotone order.
    // Assertion: keyFrame_ is not empty

    // TODO: Special case for loops when closed path is implemented !!
    if (!currentFrameValid_) {
        // Recompute everything from scrach
        currentFrame_[1]->toFront();
    }

    while (currentFrame_[1]->peekNext()->time() > time) {
        currentFrameValid_ = false;
        if (!currentFrame_[1]->hasPrevious()) {
            break;
        }

        currentFrame_[1]->previous();
    }

    if (!currentFrameValid_) {
        *currentFrame_[2] = *currentFrame_[1];
    }

    while (currentFrame_[2]->peekNext()->time() < time) {
        currentFrameValid_ = false;
        if (!currentFrame_[2]->hasNext()) {
            break;
        }

        currentFrame_[2]->next();
    }

    if (!currentFrameValid_) {
        *currentFrame_[1] = *currentFrame_[2];
        if ((currentFrame_[1]->hasPrevious()) &&
            (time < currentFrame_[2]->peekNext()->time())) {
            currentFrame_[1]->previous();
        }

        *currentFrame_[0] = *currentFrame_[1];
        if (currentFrame_[0]->hasPrevious()) {
            currentFrame_[0]->previous();
        }

        *currentFrame_[3] = *currentFrame_[2];
        if (currentFrame_[3]->hasNext()) {
            currentFrame_[3]->next();
        }

        currentFrameValid_ = true;
        splineCacheIsValid_ = false;
    }

    // cout << "Time = " << time << " : " <<
    // currentFrame_[0]->peekNext()->time()
    // << " , " << currentFrame_[1]->peekNext()->time() << " , " <<
    // currentFrame_[2]->peekNext()->time() << " , " <<
    // currentFrame_[3]->peekNext()->time() << endl;
}

void KeyFrameInterpolator::updateSplineCache()
{
    Vec delta = currentFrame_[2]->peekNext()->position() -
                currentFrame_[1]->peekNext()->position();
    v1_ = 3.0 * delta - 2.0 * currentFrame_[1]->peekNext()->tgP() -
          currentFrame_[2]->peekNext()->tgP();
    v2_ = -2.0 * delta + currentFrame_[1]->peekNext()->tgP() +
          currentFrame_[2]->peekNext()->tgP();
    splineCacheIsValid_ = true;
}

void KeyFrameInterpolator::interpolateAtTime(qreal time)
{
    setInterpolationTime(time);

    if ((keyFrames_.isEmpty()) || (!frame())) {
        return;
    }

    if (!valuesAreValid_) {
        updateModifiedFrameValues();
    }

    updateCurrentKeyFrameForTime(time);

    if (!splineCacheIsValid_) {
        updateSplineCache();
    }

    qreal alpha;
    qreal dt = currentFrame_[2]->peekNext()->time() -
               currentFrame_[1]->peekNext()->time();
    if (dt == 0.0) {
        alpha = 0.0;
    }
    else {
        alpha = (time - currentFrame_[1]->peekNext()->time()) / dt;
    }

    // Linear interpolation - debug
    // Vec pos = alpha*(currentFrame_[2]->peekNext()->position()) +
    // (1.0-alpha)*(currentFrame_[1]->peekNext()->position());
    Vec pos = currentFrame_[1]->peekNext()->position() +
              alpha * (currentFrame_[1]->peekNext()->tgP() +
                       alpha * (v1_ + alpha * v2_));
    const auto q =
        Quaternion::squad(currentFrame_[1]->peekNext()->orientation(),
                          currentFrame_[1]->peekNext()->tgQ(),
                          currentFrame_[2]->peekNext()->tgQ(),
                          currentFrame_[2]->peekNext()->orientation(), alpha);
    frame()->setPositionAndOrientationWithConstraint(pos, q);

    emit interpolated();
}

namespace key {
constexpr char kKeyFrame[]{"KeyFrame"};
constexpr char kKeyFrameIndex[]{"index"};
constexpr char kKeyFrameTime[]{"time"};
constexpr char kKeyFrameCount[]{"nbKF"};
constexpr char kInterpolationTime[]{"time"};
constexpr char kInterpolationSpeed[]{"speed"};
constexpr char kInterpolationPeriod[]{"period"};
constexpr char kClosdPath[]{"closedPath"};
constexpr char kLoopInterpolation[]{"loop"};
} // namespace key

QDomElement KeyFrameInterpolator::domElement(const QString &name,
                                             QDomDocument &document) const
{
    auto elem = document.createElement(name);

    int count = 0;
    for (const auto *kf : keyFrames_) {
        const Frame frame{kf->position(), kf->orientation()};
        auto kfNode = frame.domElement(key::kKeyFrame, document);
        kfNode.setAttribute(key::kKeyFrameIndex, QString::number(count));
        kfNode.setAttribute(key::kKeyFrameTime, QString::number(kf->time()));
        elem.appendChild(kfNode);
        ++count;
    }

    elem.setAttribute(key::kKeyFrameCount, QString::number(keyFrames_.count()));
    elem.setAttribute(key::kInterpolationTime,
                      QString::number(interpolationTime()));
    elem.setAttribute(key::kInterpolationSpeed,
                      QString::number(interpolationSpeed()));
    elem.setAttribute(key::kInterpolationPeriod,
                      QString::number(interpolationPeriod()));
    io::setBoolAttribute(elem, key::kClosdPath, closedPath());
    io::setBoolAttribute(elem, key::kLoopInterpolation, loopInterpolation());

    return elem;
}

void KeyFrameInterpolator::initFromDOMElement(const QDomElement &element)
{
    qDeleteAll(keyFrames_);
    keyFrames_.clear();

    auto child = element.firstChild().toElement();
    while (!child.isNull()) {
        if (child.tagName() == key::kKeyFrame) {
            Frame frame;
            frame.initFromDOMElement(child);
            addKeyFrame(frame, io::qrealFromDom(child, key::kKeyFrameTime, 0.));
        }

        child = child.nextSibling().toElement();
    }

    setInterpolationTime(
        io::qrealFromDom(element, key::kInterpolationTime, 0.));
    setInterpolationSpeed(
        io::qrealFromDom(element, key::kInterpolationSpeed, 1.));
    setInterpolationPeriod(io::intFromDom(element, key::kInterpolationPeriod,
                                          kDefaultInterpolationPeriod));
    setClosedPath(io::boolFromDom(element, key::kClosdPath, false));
    setLoopInterpolation(
        io::boolFromDom(element, key::kLoopInterpolation, false));

    // setFrame(nullptr);
    pathIsValid_ = false;
    valuesAreValid_ = false;
    currentFrameValid_ = false;

    stopInterpolation();
}
