#include "quaternion.h"
#include "dom_utils.h"

#include <stdlib.h> // for RAND_MAX

using namespace viewer;

Quaternion::Quaternion(const Vec &from, const Vec &to)
{
    const qreal epsilon = 1e-10;

    const qreal fromSqNorm = from.squaredNorm();
    const qreal toSqNorm = to.squaredNorm();
    // Identity Quaternion when one vector is null
    if ((fromSqNorm < epsilon) || (toSqNorm < epsilon)) {
        q[0] = q[1] = q[2] = 0.0;
        q[3] = 1.0;
    }
    else {
        Vec axis = cross(from, to);
        const qreal axisSqNorm = axis.squaredNorm();

        // Aligned vectors, pick any axis, not aligned with from or to
        if (axisSqNorm < epsilon) {
            axis = from.orthogonalVec();
        }

        qreal angle = asin(sqrt(axisSqNorm / (fromSqNorm * toSqNorm)));

        if (from * to < 0.0) {
            angle = M_PI - angle;
        }

        setAxisAngle(axis, angle);
    }
}

Vec Quaternion::inverseRotate(const Vec &v) const
{
    return inverse().rotate(v);
}

Vec Quaternion::rotate(const Vec &v) const
{
    const qreal q00 = 2.0 * q[0] * q[0];
    const qreal q11 = 2.0 * q[1] * q[1];
    const qreal q22 = 2.0 * q[2] * q[2];

    const qreal q01 = 2.0 * q[0] * q[1];
    const qreal q02 = 2.0 * q[0] * q[2];
    const qreal q03 = 2.0 * q[0] * q[3];

    const qreal q12 = 2.0 * q[1] * q[2];
    const qreal q13 = 2.0 * q[1] * q[3];

    const qreal q23 = 2.0 * q[2] * q[3];

    return {(1.0 - q11 - q22) * v[0] + (q01 - q23) * v[1] + (q02 + q13) * v[2],
            (q01 + q23) * v[0] + (1.0 - q22 - q00) * v[1] + (q12 - q03) * v[2],
            (q02 - q13) * v[0] + (q12 + q03) * v[1] + (1.0 - q11 - q00) * v[2]};
}

void Quaternion::setFromRotationMatrix(const qreal m[3][3])
{
    // Compute one plus the trace of the matrix
    const qreal onePlusTrace = 1.0 + m[0][0] + m[1][1] + m[2][2];

    if (onePlusTrace > 1e-5) {
        // Direct computation
        const qreal s = sqrt(onePlusTrace) * 2.0;
        q[0] = (m[2][1] - m[1][2]) / s;
        q[1] = (m[0][2] - m[2][0]) / s;
        q[2] = (m[1][0] - m[0][1]) / s;
        q[3] = 0.25 * s;
    }
    else {
        // Computation depends on major diagonal term
        if ((m[0][0] > m[1][1]) & (m[0][0] > m[2][2])) {
            const qreal s = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
            q[0] = 0.25 * s;
            q[1] = (m[0][1] + m[1][0]) / s;
            q[2] = (m[0][2] + m[2][0]) / s;
            q[3] = (m[1][2] - m[2][1]) / s;
        }
        else if (m[1][1] > m[2][2]) {
            const qreal s = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
            q[0] = (m[0][1] + m[1][0]) / s;
            q[1] = 0.25 * s;
            q[2] = (m[1][2] + m[2][1]) / s;
            q[3] = (m[0][2] - m[2][0]) / s;
        }
        else {
            const qreal s = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
            q[0] = (m[0][2] + m[2][0]) / s;
            q[1] = (m[1][2] + m[2][1]) / s;
            q[2] = 0.25 * s;
            q[3] = (m[0][1] - m[1][0]) / s;
        }
    }
    normalize();
}

void Quaternion::setFromRotatedBasis(const Vec &X, const Vec &Y, const Vec &Z)
{
    qreal m[3][3];
    qreal normX = X.norm();
    qreal normY = Y.norm();
    qreal normZ = Z.norm();

    for (int i = 0; i < 3; ++i) {
        m[i][0] = X[i] / normX;
        m[i][1] = Y[i] / normY;
        m[i][2] = Z[i] / normZ;
    }

    setFromRotationMatrix(m);
}

void Quaternion::getAxisAngle(Vec &axis, qreal &angle) const
{
    angle = 2.0 * acos(q[3]);
    axis = Vec(q[0], q[1], q[2]);
    const qreal sinus = axis.norm();
    if (sinus > 1e-8) {
        axis /= sinus;
    }

    if (angle > M_PI) {
        angle = 2.0 * qreal(M_PI) - angle;
        axis = -axis;
    }
}

Vec Quaternion::axis() const
{
    Vec res{q[0], q[1], q[2]};
    const qreal sinus = res.norm();
    if (sinus > 1e-8) {
        res /= sinus;
    }

    return (acos(q[3]) <= M_PI / 2.0) ? res : -res;
}

qreal Quaternion::angle() const
{
    const qreal angle = 2.0 * acos(q[3]);
    return (angle <= M_PI) ? angle : 2.0 * M_PI - angle;
}

namespace key {
constexpr char kQ0[]{"q0"};
constexpr char kQ1[]{"q1"};
constexpr char kQ2[]{"q2"};
constexpr char kQ3[]{"q3"};

} // namespace key

QDomElement Quaternion::domElement(const QString &name, QDomDocument &doc) const
{
    auto elem = doc.createElement(name);
    elem.setAttribute(key::kQ0, QString::number(q[0]));
    elem.setAttribute(key::kQ1, QString::number(q[1]));
    elem.setAttribute(key::kQ2, QString::number(q[2]));
    elem.setAttribute(key::kQ3, QString::number(q[3]));
    return elem;
}

void Quaternion::initFromDOMElement(const QDomElement &element)
{
    Quaternion q(element);
    *this = q;
}

Quaternion::Quaternion(const QDomElement &element)
{
    QStringList attribute;
    attribute << key::kQ0 << key::kQ1 << key::kQ2 << key::kQ3;
    for (int i = 0; i < attribute.size(); ++i) {
        q[i] = io::qrealFromDom(element, attribute[i], ((i < 3) ? 0.0 : 1.0));
    }
}

const GLdouble *Quaternion::matrix() const
{
    static GLdouble m[4][4];
    getMatrix(m);
    return (const GLdouble *)(m);
}

void Quaternion::getMatrix(GLdouble m[4][4]) const
{
    const qreal q00 = 2.0 * q[0] * q[0];
    const qreal q11 = 2.0 * q[1] * q[1];
    const qreal q22 = 2.0 * q[2] * q[2];

    const qreal q01 = 2.0 * q[0] * q[1];
    const qreal q02 = 2.0 * q[0] * q[2];
    const qreal q03 = 2.0 * q[0] * q[3];

    const qreal q12 = 2.0 * q[1] * q[2];
    const qreal q13 = 2.0 * q[1] * q[3];

    const qreal q23 = 2.0 * q[2] * q[3];

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

    m[3][0] = 0.0;
    m[3][1] = 0.0;
    m[3][2] = 0.0;
    m[3][3] = 1.0;
}

void Quaternion::getMatrix(GLdouble m[16]) const
{
    static GLdouble mat[4][4];
    getMatrix(mat);
    int count = 0;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            m[count++] = mat[i][j];
        }
    }
}

void Quaternion::getRotationMatrix(qreal m[3][3]) const
{
    static GLdouble mat[4][4];
    getMatrix(mat);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            // Beware of transposition
            m[i][j] = qreal(mat[j][i]);
        }
    }
}

const GLdouble *Quaternion::inverseMatrix() const
{
    static GLdouble m[4][4];
    getInverseMatrix(m);
    return (const GLdouble *)(m);
}

void Quaternion::getInverseMatrix(GLdouble m[4][4]) const
{
    inverse().getMatrix(m);
}

void Quaternion::getInverseMatrix(GLdouble m[16]) const
{
    inverse().getMatrix(m);
}

void Quaternion::getInverseRotationMatrix(qreal m[3][3]) const
{
    static GLdouble mat[4][4];
    getInverseMatrix(mat);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            // Beware of transposition
            m[i][j] = qreal(mat[j][i]);
        }
    }
}

Quaternion Quaternion::slerp(const Quaternion &a, const Quaternion &b, qreal t,
                             bool allowFlip)
{
    qreal cosAngle = Quaternion::dot(a, b);

    qreal c1, c2;
    // Linear interpolation for close orientations
    if ((1.0 - std::abs(cosAngle)) < 0.01) {
        c1 = 1.0 - t;
        c2 = t;
    }
    else {
        // Spherical interpolation
        qreal angle = std::acos(std::abs(cosAngle));
        qreal sinAngle = sin(angle);
        c1 = sin(angle * (1.0 - t)) / sinAngle;
        c2 = sin(angle * t) / sinAngle;
    }

    // Use the shortest path
    if (allowFlip && (cosAngle < 0.0)) {
        c1 = -c1;
    }

    return {c1 * a[0] + c2 * b[0], c1 * a[1] + c2 * b[1], c1 * a[2] + c2 * b[2],
            c1 * a[3] + c2 * b[3]};
}

Quaternion Quaternion::squad(const Quaternion &a, const Quaternion &tgA,
                             const Quaternion &tgB, const Quaternion &b,
                             qreal t)
{
    const auto ab = Quaternion::slerp(a, b, t);
    const auto tg = Quaternion::slerp(tgA, tgB, t, false);
    return Quaternion::slerp(ab, tg, 2.0 * t * (1.0 - t), false);
}

Quaternion Quaternion::log()
{
    qreal len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]);
    if (len < 1e-6) {
        return {q[0], q[1], q[2], 0.};
    }

    qreal coef = std::acos(q[3]) / len;
    return {q[0] * coef, q[1] * coef, q[2] * coef, 0.};
}

Quaternion Quaternion::exp()
{
    qreal theta = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]);
    if (theta < 1e-6) {
        return {q[0], q[1], q[2], cos(theta)};
    }

    qreal coef = sin(theta) / theta;
    return {q[0] * coef, q[1] * coef, q[2] * coef, cos(theta)};
}

Quaternion Quaternion::lnDif(const Quaternion &a, const Quaternion &b)
{
    Quaternion dif = a.inverse() * b;
    dif.normalize();
    return dif.log();
}

Quaternion Quaternion::squadTangent(const Quaternion &before,
                                    const Quaternion &center,
                                    const Quaternion &after)
{
    const auto l1 = Quaternion::lnDif(center, before);
    const auto l2 = Quaternion::lnDif(center, after);

    Quaternion e;
    for (int i = 0; i < Size; ++i) {
        e.q[i] = -0.25 * (l1.q[i] + l2.q[i]);
    }
    e = center * (e.exp());

    // if (Quaternion::dot(e,b) < 0.0)
    // e.negate();

    return e;
}

std::ostream &operator<<(std::ostream &os, const Quaternion &Q)
{
    return os << Q[0] << '\t' << Q[1] << '\t' << Q[2] << '\t' << Q[3];
}

Quaternion Quaternion::random()
{
    // The rand() function is not very portable and may not be available on your
    // system. Add the appropriate include or replace by an other random
    // function in case of problem.
    qreal seed = rand() / (qreal)RAND_MAX;
    qreal r1 = sqrt(1.0 - seed);
    qreal r2 = sqrt(seed);
    qreal t1 = 2.0 * M_PI * (rand() / (qreal)RAND_MAX);
    qreal t2 = 2.0 * M_PI * (rand() / (qreal)RAND_MAX);
    return Quaternion(sin(t1) * r1, cos(t1) * r1, sin(t2) * r2, cos(t2) * r2);
}
