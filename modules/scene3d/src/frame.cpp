#include "frame.h"

#include <cmath>

using namespace viewer;

Frame::Frame() : constraint_(nullptr), referenceFrame_(nullptr) {}

Frame::Frame(const Vec &position, const Quaternion &orientation)
    : t_(position),
      q_(orientation),
      constraint_(nullptr),
      referenceFrame_(nullptr)
{
}

Frame &Frame::operator=(const Frame &frame)
{
    // Automatic compiler generated version would not emit the modified()
    // signals as is done in setTranslationAndRotation.
    setTranslationAndRotation(frame.translation(), frame.rotation());
    setConstraint(frame.constraint());
    setReferenceFrame(frame.referenceFrame());
    return *this;
}

Frame::Frame(const Frame &frame) : QObject() { (*this) = frame; }

const GLdouble *Frame::matrix() const
{
    static GLdouble m[4][4];
    getMatrix(m);
    return (const GLdouble *)(m);
}

void Frame::getMatrix(GLdouble m[4][4]) const
{
    q_.getMatrix(m);
    m[3][0] = t_[0];
    m[3][1] = t_[1];
    m[3][2] = t_[2];
}

void Frame::getMatrix(GLdouble m[16]) const
{
    q_.getMatrix(m);

    m[12] = t_[0];
    m[13] = t_[1];
    m[14] = t_[2];
}

Frame Frame::inverse() const
{
    Frame fr(-(q_.inverseRotate(t_)), q_.inverse());
    fr.setReferenceFrame(referenceFrame());
    return fr;
}

const GLdouble *Frame::worldMatrix() const
{
    // This test is done for efficiency reasons (creates lots of temp objects
    // otherwise).
    if (referenceFrame()) {
        static Frame fr;
        fr.setTranslation(position());
        fr.setRotation(orientation());
        return fr.matrix();
    }

    return matrix();
}

void Frame::getWorldMatrix(GLdouble m[4][4]) const
{
    const GLdouble *mat = worldMatrix();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            m[i][j] = mat[i * 4 + j];
        }
    }
}

void Frame::getWorldMatrix(GLdouble m[16]) const
{
    const GLdouble *mat = worldMatrix();
    for (int i = 0; i < 16; ++i) {
        m[i] = mat[i];
    }
}

void Frame::setFromMatrix(const GLdouble m[4][4])
{
    if (fabs(m[3][3]) < 1e-8) {
        qWarning("Frame::setFromMatrix: Null homogeneous coefficient");
        return;
    }

    qreal rot[3][3];
    for (int i = 0; i < 3; ++i) {
        t_[i] = m[3][i] / m[3][3];
        for (int j = 0; j < 3; ++j) {
            // Beware of the transposition (OpenGL to European math)
            rot[i][j] = m[j][i] / m[3][3];
        }
    }
    q_.setFromRotationMatrix(rot);
    emit modified();
}

void Frame::setFromMatrix(const GLdouble m[16])
{
    GLdouble mat[4][4];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            mat[i][j] = m[i * 4 + j];
        }
    }
    setFromMatrix(mat);
}

void Frame::setTranslation(const Vec &translation)
{
    t_ = translation;
    emit modified();
}

void Frame::setTranslationWithConstraint(const Vec &translation)
{
    Vec deltaT = translation - this->translation();
    if (constraint()) {
        constraint()->constrainTranslation(deltaT, this);
    }

    setTranslation(this->translation() + deltaT);
}

void Frame::getTranslation(qreal &x, qreal &y, qreal &z) const
{
    const Vec t = translation();
    x = t[0];
    y = t[1];
    z = t[2];
}

void Frame::setRotation(const Quaternion &rotation)
{
    q_ = rotation;
    emit modified();
}

void Frame::setRotationWithConstraint(const Quaternion &rotation)
{
    Quaternion deltaQ = this->rotation().inverse() * rotation;
    if (constraint()) {
        constraint()->constrainRotation(deltaQ, this);
    }

    // Prevent numerical drift
    deltaQ.normalize();

    setRotation(this->rotation() * deltaQ);
    q_.normalize();
}

Vec Frame::translation() const { return t_; }

Quaternion Frame::rotation() const { return q_; }

void Frame::getRotation(qreal &q0, qreal &q1, qreal &q2, qreal &q3) const
{
    const Quaternion q = rotation();
    q0 = q[0];
    q1 = q[1];
    q2 = q[2];
    q3 = q[3];
}

Constraint *Frame::constraint() const { return constraint_; }

void Frame::setConstraint(Constraint *const constraint)
{
    constraint_ = constraint;
}

Frame Frame::worldInverse() const
{
    return {-orientation().inverseRotate(position()), orientation().inverse()};
}

void Frame::translate(const Vec &t)
{
    Vec tbis = t;
    translate(tbis);
}

void Frame::translate(Vec &t)
{
    if (constraint()) {
        constraint()->constrainTranslation(t, this);
    }

    t_ += t;
    emit modified();
}

void Frame::translate(qreal x, qreal y, qreal z)
{
    Vec t(x, y, z);
    translate(t);
}

void Frame::translate(qreal &x, qreal &y, qreal &z)
{
    Vec t(x, y, z);
    translate(t);
    x = t[0];
    y = t[1];
    z = t[2];
}

void Frame::rotate(const Quaternion &q)
{
    Quaternion qbis = q;
    rotate(qbis);
}

void Frame::rotate(Quaternion &q)
{
    if (constraint()) {
        constraint()->constrainRotation(q, this);
    }

    q_ *= q;
    q_.normalize(); // Prevents numerical drift
    emit modified();
}

void Frame::rotate(qreal &q0, qreal &q1, qreal &q2, qreal &q3)
{
    Quaternion q(q0, q1, q2, q3);
    rotate(q);
    q0 = q[0];
    q1 = q[1];
    q2 = q[2];
    q3 = q[3];
}

void Frame::rotate(qreal q0, qreal q1, qreal q2, qreal q3)
{
    Quaternion q(q0, q1, q2, q3);
    rotate(q);
}

void Frame::rotateAroundPoint(Quaternion &rotation, const Vec &point)
{
    if (constraint())
        constraint()->constrainRotation(rotation, this);
    q_ *= rotation;
    q_.normalize(); // Prevents numerical drift
    Vec trans =
        point +
        Quaternion(inverseTransformOf(rotation.axis()), rotation.angle())
            .rotate(position() - point) -
        t_;
    if (constraint())
        constraint()->constrainTranslation(trans, this);
    t_ += trans;
    emit modified();
}

void Frame::rotateAroundPoint(const Quaternion &rotation, const Vec &point)
{
    Quaternion rot = rotation;
    rotateAroundPoint(rot, point);
}

void Frame::setPosition(const Vec &position)
{
    setTranslation(referenceFrame() ? referenceFrame()->coordinatesOf(position)
                                    : position);
}

void Frame::setPositionWithConstraint(const Vec &position)
{
    Vec pos = position;
    if (referenceFrame()) {
        pos = referenceFrame()->coordinatesOf(position);
    }

    setTranslationWithConstraint(pos);
}

void Frame::setPositionAndOrientation(const Vec &position,
                                      const Quaternion &orientation)
{
    if (referenceFrame()) {
        t_ = referenceFrame()->coordinatesOf(position);
        q_ = referenceFrame()->orientation().inverse() * orientation;
    }
    else {
        t_ = position;
        q_ = orientation;
    }
    emit modified();
}

void Frame::setTranslationAndRotation(const Vec &translation,
                                      const Quaternion &rotation)
{
    t_ = translation;
    q_ = rotation;
    emit modified();
}

void Frame::getPosition(qreal &x, qreal &y, qreal &z) const
{
    Vec p = position();
    x = p.x;
    y = p.y;
    z = p.z;
}

void Frame::setOrientation(const Quaternion &orientation)
{
    setRotation(referenceFrame()
                    ? referenceFrame()->orientation().inverse() * orientation
                    : orientation);
}

void Frame::setOrientation(qreal x, qreal y, qreal z, qreal w)
{
    setOrientation({x, y, z, w});
}

void Frame::getOrientation(qreal &q0, qreal &q1, qreal &q2, qreal &q3) const
{
    Quaternion o = orientation();
    q0 = o[0];
    q1 = o[1];
    q2 = o[2];
    q3 = o[3];
}

Vec Frame::position() const
{
    if (referenceFrame_)
        return inverseCoordinatesOf(Vec(0.0, 0.0, 0.0));

    return t_;
}

Quaternion Frame::orientation() const
{
    Quaternion res = rotation();
    const Frame *fr = referenceFrame();
    while (fr) {
        res = fr->rotation() * res;
        fr = fr->referenceFrame();
    }
    return res;
}

void Frame::setTranslationAndRotationWithConstraint(const Vec &translation,
                                                    const Quaternion &rotation)
{
    Vec deltaT = translation - this->translation();
    Quaternion deltaQ = this->rotation().inverse() * rotation;

    if (constraint()) {
        constraint()->constrainTranslation(deltaT, this);
        constraint()->constrainRotation(deltaQ, this);
    }

    // Prevent numerical drift
    deltaQ.normalize();

    t_ += deltaT;
    q_ *= deltaQ;
    q_.normalize();

    emit modified();
}

void Frame::setOrientationWithConstraint(const Quaternion &orientation)
{
    setRotationWithConstraint(referenceFrame()
                                  ? referenceFrame()->orientation().inverse() *
                                        orientation
                                  : orientation);
}

void Frame::setPositionAndOrientationWithConstraint(
    const Vec &position, const Quaternion &orientation)
{
    Vec pos = position;
    Quaternion ori = orientation;
    if (referenceFrame()) {
        pos = referenceFrame()->coordinatesOf(position);
        ori = referenceFrame()->orientation().inverse() * orientation;
    }
    setTranslationAndRotationWithConstraint(pos, ori);
}

void Frame::setReferenceFrame(const Frame *const refFrame)
{
    if (settingAsReferenceFrameWillCreateALoop(refFrame)) {
        qWarning(
            "Frame::setReferenceFrame would create a loop in Frame hierarchy");
    }
    else {
        bool identical = (referenceFrame_ == refFrame);
        referenceFrame_ = refFrame;
        if (!identical) {
            emit modified();
        }
    }
}

bool Frame::settingAsReferenceFrameWillCreateALoop(const Frame *const frame)
{
    const Frame *f = frame;
    while (f) {
        if (f == this) {
            return true;
        }

        f = f->referenceFrame();
    }
    return false;
}

Vec Frame::coordinatesOf(const Vec &src) const
{
    if (referenceFrame()) {
        return localCoordinatesOf(referenceFrame()->coordinatesOf(src));
    }

    return localCoordinatesOf(src);
}

Vec Frame::inverseCoordinatesOf(const Vec &src) const
{
    const Frame *fr = this;
    Vec res = src;
    while (fr) {
        res = fr->localInverseCoordinatesOf(res);
        fr = fr->referenceFrame();
    }
    return res;
}

Vec Frame::localCoordinatesOf(const Vec &src) const
{
    return rotation().inverseRotate(src - translation());
}

Vec Frame::localInverseCoordinatesOf(const Vec &src) const
{
    return rotation().rotate(src) + translation();
}

Vec Frame::coordinatesOfFrom(const Vec &src, const Frame *const from) const
{
    if (this == from) {
        return src;
    }

    if (referenceFrame()) {
        return localCoordinatesOf(
            referenceFrame()->coordinatesOfFrom(src, from));
    }

    return localCoordinatesOf(from->inverseCoordinatesOf(src));
}

Vec Frame::coordinatesOfIn(const Vec &src, const Frame *const in) const
{
    const Frame *fr = this;
    Vec res = src;
    while (fr && (fr != in)) {
        res = fr->localInverseCoordinatesOf(res);
        fr = fr->referenceFrame();
    }

    if (fr != in)
        // in was not found in the branch of this, res is now expressed in the
        // world coordinate system. Simply convert to in coordinate system.
        res = in->coordinatesOf(res);

    return res;
}

void Frame::getCoordinatesOf(const qreal src[3], qreal res[3]) const
{
    const Vec r = coordinatesOf(Vec(src));
    for (int i = 0; i < 3; ++i) {
        res[i] = r[i];
    }
}

void Frame::getInverseCoordinatesOf(const qreal src[3], qreal res[3]) const
{
    const Vec r = inverseCoordinatesOf(Vec(src));
    for (int i = 0; i < 3; ++i) {
        res[i] = r[i];
    }
}

void Frame::getLocalCoordinatesOf(const qreal src[3], qreal res[3]) const
{
    const Vec r = localCoordinatesOf(Vec(src));
    for (int i = 0; i < 3; ++i) {
        res[i] = r[i];
    }
}

void Frame::getLocalInverseCoordinatesOf(const qreal src[3], qreal res[3]) const
{
    const Vec r = localInverseCoordinatesOf(Vec(src));
    for (int i = 0; i < 3; ++i) {
        res[i] = r[i];
    }
}

void Frame::getCoordinatesOfIn(const qreal src[3], qreal res[3],
                               const Frame *const in) const
{
    const Vec r = coordinatesOfIn(Vec(src), in);
    for (int i = 0; i < 3; ++i) {
        res[i] = r[i];
    }
}

void Frame::getCoordinatesOfFrom(const qreal src[3], qreal res[3],
                                 const Frame *const from) const
{
    const Vec r = coordinatesOfFrom(Vec(src), from);
    for (int i = 0; i < 3; ++i) {
        res[i] = r[i];
    }
}

Vec Frame::transformOf(const Vec &src) const
{
    if (referenceFrame())
        return localTransformOf(referenceFrame()->transformOf(src));
    return localTransformOf(src);
}

Vec Frame::inverseTransformOf(const Vec &src) const
{
    const Frame *fr = this;
    Vec res = src;
    while (fr) {
        res = fr->localInverseTransformOf(res);
        fr = fr->referenceFrame();
    }
    return res;
}

Vec Frame::localTransformOf(const Vec &src) const
{
    return rotation().inverseRotate(src);
}

Vec Frame::localInverseTransformOf(const Vec &src) const
{
    return rotation().rotate(src);
}

Vec Frame::transformOfFrom(const Vec &src, const Frame *const from) const
{
    if (this == from) {
        return src;
    }
    if (referenceFrame()) {
        return localTransformOf(referenceFrame()->transformOfFrom(src, from));
    }

    return localTransformOf(from->inverseTransformOf(src));
}

Vec Frame::transformOfIn(const Vec &src, const Frame *const in) const
{
    const Frame *fr = this;
    Vec res = src;
    while (fr && (fr != in)) {
        res = fr->localInverseTransformOf(res);
        fr = fr->referenceFrame();
    }

    if (fr != in)
        // in was not found in the branch of this, res is now expressed in the
        // world coordinate system. Simply convert to in coordinate system.
        res = in->transformOf(res);

    return res;
}

void Frame::getTransformOf(const qreal src[3], qreal res[3]) const
{
    Vec r = transformOf(Vec(src));
    for (int i = 0; i < 3; ++i) {
        res[i] = r[i];
    }
}

void Frame::getInverseTransformOf(const qreal src[3], qreal res[3]) const
{
    Vec r = inverseTransformOf(Vec(src));
    for (int i = 0; i < 3; ++i) {
        res[i] = r[i];
    }
}

void Frame::getLocalTransformOf(const qreal src[3], qreal res[3]) const
{
    Vec r = localTransformOf(Vec(src));
    for (int i = 0; i < 3; ++i) {
        res[i] = r[i];
    }
}

void Frame::getLocalInverseTransformOf(const qreal src[3], qreal res[3]) const
{
    Vec r = localInverseTransformOf(Vec(src));
    for (int i = 0; i < 3; ++i) {
        res[i] = r[i];
    }
}

void Frame::getTransformOfIn(const qreal src[3], qreal res[3],
                             const Frame *const in) const
{
    Vec r = transformOfIn(Vec(src), in);
    for (int i = 0; i < 3; ++i) {
        res[i] = r[i];
    }
}

void Frame::getTransformOfFrom(const qreal src[3], qreal res[3],
                               const Frame *const from) const
{
    Vec r = transformOfFrom(Vec(src), from);
    for (int i = 0; i < 3; ++i) {
        res[i] = r[i];
    }
}

namespace key {
constexpr char kPosition[]{"position"};
constexpr char kOrientation[]{"orientation"};
} // namespace key

QDomElement Frame::domElement(const QString &name, QDomDocument &document) const
{
    // TODO: use translation and rotation instead when referenceFrame is
    // coded...
    auto e = document.createElement(name);
    e.appendChild(position().domElement(key::kPosition, document));
    e.appendChild(orientation().domElement(key::kOrientation, document));
    return e;
}

void Frame::initFromDOMElement(const QDomElement &element)
{
    // TODO: use translation and rotation instead when referenceFrame is
    // coded...

    // Reset default values. Attention: destroys constraint.
    // *this = Frame();
    // This instead ? Better : what is not set is not changed.
    // setPositionAndOrientation(Vec(), Quaternion());

    auto child = element.firstChild().toElement();
    while (!child.isNull()) {
        if (child.tagName() == key::kPosition) {
            setPosition(Vec(child));
        }
        if (child.tagName() == key::kOrientation) {
            setOrientation(Quaternion(child).normalized());
        }

        child = child.nextSibling().toElement();
    }
}

void Frame::alignWithFrame(const Frame *const frame, bool move, qreal threshold)
{
    Vec directions[2][3];
    for (int d{0}; d < 3; ++d) {
        Vec dir{(d == 0) ? 1. : 0., (d == 1) ? 1. : 0., (d == 2) ? 1. : 0.};
        if (frame)
            directions[0][d] = frame->inverseTransformOf(dir);
        else
            directions[0][d] = dir;
        directions[1][d] = inverseTransformOf(dir);
    }

    qreal maxProj = 0.0;
    qreal proj;
    unsigned short index[2];
    index[0] = index[1] = 0;
    for (int i{0}; i < 3; ++i) {
        for (int j{0}; j < 3; ++j) {
            if ((proj = fabs(directions[0][i] * directions[1][j])) >= maxProj) {
                index[0] = i;
                index[1] = j;
                maxProj = proj;
            }
        }
    }

    Frame old;
    old = *this;

    qreal coef = directions[0][index[0]] * directions[1][index[1]];
    if (fabs(coef) >= threshold) {
        const Vec axis =
            cross(directions[0][index[0]], directions[1][index[1]]);
        qreal angle = asin(axis.norm());
        if (coef >= 0.0) {
            angle = -angle;
        }
        rotate(rotation().inverse() * Quaternion(axis, angle) * orientation());

        // Try to align an other axis direction
        unsigned short d = (index[1] + 1) % 3;
        Vec dir{(d == 0) ? 1. : 0., (d == 1) ? 1. : 0., (d == 2) ? 1. : 0.};
        dir = inverseTransformOf(dir);

        qreal max{0.};
        for (int i{0}; i < 3; ++i) {
            qreal proj = fabs(directions[0][i] * dir);
            if (proj > max) {
                index[0] = i;
                max = proj;
            }
        }

        if (max >= threshold) {
            const Vec axis = cross(directions[0][index[0]], dir);
            qreal angle = asin(axis.norm());
            if (directions[0][index[0]] * dir >= 0.0) {
                angle = -angle;
            }
            rotate(rotation().inverse() * Quaternion(axis, angle) *
                   orientation());
        }
    }

    if (move) {
        Vec center;
        if (frame) {
            center = frame->position();
        }

        translate(center - orientation().rotate(old.coordinatesOf(center)) -
                  translation());
    }
}

void Frame::projectOnLine(const Vec &origin, const Vec &direction)
{
    // If you are trying to find a bug here, because of memory problems, you
    // waste your time. This is a bug in the gcc 3.3 compiler. Compile the
    // library in debug mode and test. Uncommenting this line also seems to
    // solve the problem. Horrible. cout << "position = " << position() << endl;
    // If you found a problem or are using a different compiler, please let me
    // know.
    const Vec shift = origin - position();
    Vec proj = shift;
    proj.projectOnAxis(direction);
    translate(shift - proj);
}
