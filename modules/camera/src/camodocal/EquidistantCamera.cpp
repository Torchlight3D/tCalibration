#include "EquidistantCamera.h"

#include <iostream>

#include <Eigen/Eigenvalues>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#include "gpl.h"

namespace camodocal {

EquidistantCamera::Parameters::Parameters()
    : Camera::Parameters(KANNALA_BRANDT),
      m_k2(0.0),
      m_k3(0.0),
      m_k4(0.0),
      m_k5(0.0),
      m_mu(0.0),
      m_mv(0.0),
      m_u0(0.0),
      m_v0(0.0)
{
}

EquidistantCamera::Parameters::Parameters(const std::string &cameraName, int w,
                                          int h, double k2, double k3,
                                          double k4, double k5, double mu,
                                          double mv, double u0, double v0)
    : Camera::Parameters(KANNALA_BRANDT, cameraName, w, h),
      m_k2(k2),
      m_k3(k3),
      m_k4(k4),
      m_k5(k5),
      m_mu(mu),
      m_mv(mv),
      m_u0(u0),
      m_v0(v0)
{
}

double &EquidistantCamera::Parameters::k2() { return m_k2; }

double &EquidistantCamera::Parameters::k3() { return m_k3; }

double &EquidistantCamera::Parameters::k4() { return m_k4; }

double &EquidistantCamera::Parameters::k5() { return m_k5; }

double &EquidistantCamera::Parameters::mu() { return m_mu; }

double &EquidistantCamera::Parameters::mv() { return m_mv; }

double &EquidistantCamera::Parameters::u0() { return m_u0; }

double &EquidistantCamera::Parameters::v0() { return m_v0; }

double EquidistantCamera::Parameters::k2() const { return m_k2; }

double EquidistantCamera::Parameters::k3() const { return m_k3; }

double EquidistantCamera::Parameters::k4() const { return m_k4; }

double EquidistantCamera::Parameters::k5() const { return m_k5; }

double EquidistantCamera::Parameters::mu() const { return m_mu; }

double EquidistantCamera::Parameters::mv() const { return m_mv; }

double EquidistantCamera::Parameters::u0() const { return m_u0; }

double EquidistantCamera::Parameters::v0() const { return m_v0; }

namespace key {
constexpr char kProjectionParameters[]{"projection_parameters"};
constexpr char kK2[]{"k2"};
constexpr char kK3[]{"k3"};
constexpr char kK4[]{"k4"};
constexpr char kK5[]{"k5"};
constexpr char kMu[]{"mu"};
constexpr char kMv[]{"mv"};
constexpr char kU0[]{"u0"};
constexpr char kV0[]{"v0"};
} // namespace key

bool EquidistantCamera::Parameters::read(const cv::FileNode &node)
{
    if (!Camera::Parameters::read(node)) {
        return false;
    }

    // Meta
    m_modelType = KANNALA_BRANDT;

    // Projection
    const auto projNode = node[key::kProjectionParameters];
    m_k2 = (double)projNode[key::kK2];
    m_k3 = (double)projNode[key::kK3];
    m_k4 = (double)projNode[key::kK4];
    m_k5 = (double)projNode[key::kK5];
    m_mu = (double)projNode[key::kMu];
    m_mv = (double)projNode[key::kMv];
    m_u0 = (double)projNode[key::kU0];
    m_v0 = (double)projNode[key::kV0];

    return true;
}

void EquidistantCamera::Parameters::write(cv::FileStorage &fs) const
{
    // Meta
    Camera::Parameters::write(fs);

    // Projection: k2, k3, k4, k5, mu, mv, u0, v0
    fs << key::kProjectionParameters;
    // clang-format off
    fs << "{"
       << key::kK2 << m_k2
       << key::kK3 << m_k3
       << key::kK4 << m_k4
       << key::kK5 << m_k5
       << key::kMu << m_mu
       << key::kMv << m_mv
       << key::kU0 << m_u0
       << key::kV0 << m_v0
       << "}";
    // clang-format on
}

EquidistantCamera::Parameters &EquidistantCamera::Parameters::operator=(
    const EquidistantCamera::Parameters &other)
{
    if (this != &other) {
        m_modelType = other.m_modelType;
        m_cameraName = other.m_cameraName;
        m_imageWidth = other.m_imageWidth;
        m_imageHeight = other.m_imageHeight;
        m_k2 = other.m_k2;
        m_k3 = other.m_k3;
        m_k4 = other.m_k4;
        m_k5 = other.m_k5;
        m_mu = other.m_mu;
        m_mv = other.m_mv;
        m_u0 = other.m_u0;
        m_v0 = other.m_v0;
    }

    return *this;
}

std::ostream &operator<<(std::ostream &out,
                         const EquidistantCamera::Parameters &params)
{
    out << "Camera Parameters:"
           "\n"
           "    model_type "
        << EquidistantCamera::Parameters::kModelTypeName
        << "\n"
           "   camera_name "
        << params.m_cameraName
        << "\n"
           "   image_width "
        << params.m_imageWidth
        << "\n"
           "  image_height "
        << params.m_imageHeight
        << "\n"

           // Projection: k2, k3, k4, k5, mu, mv, u0, v0
           "Projection Parameters"
        << "\n"
           "            k2 "
        << params.m_k2
        << "\n"
           "            k3 "
        << params.m_k3
        << "\n"
           "            k4 "
        << params.m_k4
        << "\n"
           "            k5 "
        << params.m_k5
        << "\n"
           "            mu "
        << params.m_mu
        << "\n"
           "            mv "
        << params.m_mv
        << "\n"
           "            u0 "
        << params.m_u0
        << "\n"
           "            v0 "
        << params.m_v0 << std::endl;

    return out;
}

EquidistantCamera::EquidistantCamera()
    : m_inv_K11(1.0), m_inv_K13(0.0), m_inv_K22(1.0), m_inv_K23(0.0)
{
}

EquidistantCamera::EquidistantCamera(
    const EquidistantCamera::Parameters &params)
    : mParameters(params)
{
    // Inverse camera projection matrix parameters
    m_inv_K11 = 1.0 / mParameters.mu();
    m_inv_K13 = -mParameters.u0() / mParameters.mu();
    m_inv_K22 = 1.0 / mParameters.mv();
    m_inv_K23 = -mParameters.v0() / mParameters.mv();
}

Camera::ModelType EquidistantCamera::modelType() const
{
    return mParameters.modelType();
}

const std::string &EquidistantCamera::cameraName() const
{
    return mParameters.cameraName();
}

int EquidistantCamera::imageWidth() const { return mParameters.imageWidth(); }

int EquidistantCamera::imageHeight() const { return mParameters.imageHeight(); }

void EquidistantCamera::estimateIntrinsics(
    const cv::Size &boardSize,
    const std::vector<std::vector<cv::Point3f>> &objectPoints,
    const std::vector<std::vector<cv::Point2f>> &imagePoints)
{
    Parameters params = getParameters();

    double u0 = params.imageWidth() / 2.0;
    double v0 = params.imageHeight() / 2.0;

    double minReprojErr = std::numeric_limits<double>::max();

    std::vector<cv::Mat> rvecs, tvecs;
    rvecs.assign(objectPoints.size(), cv::Mat());
    tvecs.assign(objectPoints.size(), cv::Mat());

    params.k2() = 0.0;
    params.k3() = 0.0;
    params.k4() = 0.0;
    params.k5() = 0.0;
    params.u0() = u0;
    params.v0() = v0;

    // Initialize focal length
    // C. Hughes, P. Denny, M. Glavin, and E. Jones,
    // Equidistant Fish-Eye Calibration and Rectification by Vanishing Point
    // Extraction, PAMI 2010
    // Find circles from rows of chessboard corners, and for each pair
    // of circles, find vanishing points: v1 and v2.
    // f = ||v1 - v2|| / PI;
    double f0 = 0.0;
    for (size_t i = 0; i < imagePoints.size(); ++i) {
        std::vector<Eigen::Vector2d> center(boardSize.height);
        std::vector<double> radius(boardSize.height, 0.);
        for (int r = 0; r < boardSize.height; ++r) {
            std::vector<cv::Point2d> circle;
            for (int c = 0; c < boardSize.width; ++c) {
                circle.push_back(imagePoints.at(i).at(r * boardSize.width + c));
            }

            fitCircle(circle, center[r](0), center[r](1), radius[r]);
        }

        for (int j = 0; j < boardSize.height; ++j) {
            for (int k = j + 1; k < boardSize.height; ++k) {
                // find distance between pair of vanishing points which
                // correspond to intersection points of 2 circles
                std::vector<cv::Point2d> ipts;
                ipts = intersectCircles(center[j](0), center[j](1), radius[j],
                                        center[k](0), center[k](1), radius[k]);

                if (ipts.size() < 2) {
                    continue;
                }

                double f = cv::norm(ipts.at(0) - ipts.at(1)) / pi;

                params.mu() = f;
                params.mv() = f;

                setParameters(params);

                for (size_t l = 0; l < objectPoints.size(); ++l) {
                    estimateExtrinsics(objectPoints.at(l), imagePoints.at(l),
                                       rvecs.at(l), tvecs.at(l));
                }

                double reprojErr = reprojectionError(
                    objectPoints, imagePoints, rvecs, tvecs, cv::noArray());

                if (reprojErr < minReprojErr) {
                    minReprojErr = reprojErr;
                    f0 = f;
                }
            }
        }
    }

    if (f0 <= 0.0 && minReprojErr >= std::numeric_limits<double>::max()) {
        std::cout << "[" << params.cameraName() << "] "
                  << "# INFO: kannala-Brandt model fails with given data. "
                  << std::endl;

        return;
    }

    params.mu() = f0;
    params.mv() = f0;

    setParameters(params);
}

/**
 * \brief Lifts a point from the image plane to the unit sphere
 *
 * \param p image coordinates
 * \param P coordinates of the point on the sphere
 */
void EquidistantCamera::liftSphere(const Eigen::Vector2d &p,
                                   Eigen::Vector3d &P) const
{
    liftProjective(p, P);
}

/**
 * \brief Lifts a point from the image plane to its projective ray
 *
 * \param p image coordinates
 * \param P coordinates of the projective ray
 */
void EquidistantCamera::liftProjective(const Eigen::Vector2d &p,
                                       Eigen::Vector3d &P) const
{
    // Lift points to normalised plane
    Eigen::Vector2d p_u;
    p_u << m_inv_K11 * p(0) + m_inv_K13, m_inv_K22 * p(1) + m_inv_K23;

    // Obtain a projective ray
    double theta, phi;
    backprojectSymmetric(p_u, theta, phi);

    P(0) = std::sin(theta) * std::cos(phi);
    P(1) = std::sin(theta) * std::sin(phi);
    P(2) = std::cos(theta);
}

void EquidistantCamera::liftProjectiveByIteration1(const Eigen::Vector2d &px,
                                                   Eigen::Vector3d &ray) const
{
    const auto &fx = mParameters.mu();
    const auto &fy = mParameters.mv();
    const auto &cx = mParameters.u0();
    const auto &cy = mParameters.v0();
    const auto &k1 = mParameters.k2();
    const auto &k2 = mParameters.k3();
    const auto &k3 = mParameters.k4();
    const auto &k4 = mParameters.k5();

    using Eigen::Vector2d;

    const Vector2d px_norm =
        (px - Vector2d{cx, cy}).cwiseProduct(Vector2d{1. / fx, 1. / fy});

    const auto thetad = px_norm.norm();

    constexpr int kIteration{14};
    auto theta = thetad;
    double theta2, theta4, theta8;
    for (int i{0}; i < kIteration; ++i) {
        theta2 = theta * theta;
        theta4 = theta2 * theta2;
        theta8 = theta4 * theta4;
        // clang-format off
        theta = thetad / (1.
                          + k1 * theta2
                          + k2 * theta4
                          + k3 * theta4 * theta2
                          + k4 * theta8);
        // clang-format on
    }
    const auto scale = std::tan(theta) / thetad;

    ray.head<2>() = scale * px_norm;
    ray[2] = 1.;
}

void EquidistantCamera::liftProjectiveByIteration2(const Eigen::Vector2d &px,
                                                   Eigen::Vector3d &ray) const
{
    const auto &fx = mParameters.mu();
    const auto &fy = mParameters.mv();
    const auto &cx = mParameters.u0();
    const auto &cy = mParameters.v0();
    const auto &k1 = mParameters.k2();
    const auto &k2 = mParameters.k3();
    const auto &k3 = mParameters.k4();
    const auto &k4 = mParameters.k5();

    using Eigen::Vector2d;

    const Vector2d px_norm =
        (px - Vector2d{cx, cy}).cwiseProduct(Vector2d{1. / fx, 1. / fy});

    constexpr int kIteration{100};
    constexpr double kVerySmallNumber{1e-8};
    constexpr double kUndistortionEpsilon{1e-10};

    Vector2d prev_px_u = Vector2d::Zero();
    Vector2d px_u = px_norm;

    double scale = 1.;
    double theta, theta2, theta4, theta8, thetad;
    for (int i{0}; i < kIteration; ++i) {
        prev_px_u = px_u;

        const auto psi = px_u.norm();
        if (psi < kVerySmallNumber) {
            break;
        }

        theta = std::atan2(psi, 1.);
        theta2 = theta * theta;
        theta4 = theta2 * theta2;
        theta8 = theta4 * theta4;
        // clang-format off
        thetad = theta * (1.
                          + k1 * theta2
                          + k2 * theta4
                          + k3 * theta4 * theta2
                          + k4 * theta8);
        // clang-format on
        scale = psi / thetad;
        px_u = scale * px_norm;

        if (((prev_px_u - px_u).cwiseAbs().array() < kUndistortionEpsilon)
                .all()) {
            break;
        }
    }

    ray.head<2>() = px_u;
    ray[2] = 1.;
}

/**
 * \brief Project a 3D point (\a x,\a y,\a z) to the image plane in (\a u,\a v)
 *
 * \param P 3D point coordinates
 * \param p return value, contains the image point coordinates
 */
void EquidistantCamera::spaceToPlane(const Eigen::Vector3d &P,
                                     Eigen::Vector2d &p) const
{
    double theta = acos(P(2) / P.norm());
    double phi = atan2(P(1), P(0));

    Eigen::Vector2d p_u = r(mParameters.k2(), mParameters.k3(),
                            mParameters.k4(), mParameters.k5(), theta) *
                          Eigen::Vector2d(cos(phi), sin(phi));

    // Apply generalised projection matrix
    p << mParameters.mu() * p_u(0) + mParameters.u0(),
        mParameters.mv() * p_u(1) + mParameters.v0();
}

void EquidistantCamera::spaceToPlane2(const Eigen::Vector3d &P,
                                      Eigen::Vector2d &p) const
{
    const auto &k1 = mParameters.k2();
    const auto &k2 = mParameters.k3();
    const auto &k3 = mParameters.k4();
    const auto &k4 = mParameters.k5();

    const auto rr = P.head<2>().squaredNorm();
    // Force distortion here
    // if (rr < std::numeric_limits<double>::epsilon()) {
    //     return false;
    // }

    auto rd = std::sqrt(rr);
    const auto theta = std::atan2(rd, std::abs(P(2)));
    const auto theta_2 = theta * theta;
    const auto theta_4 = theta_2 * theta_2;
    const auto theta_8 = theta_4 * theta_4;
    auto thetad = theta * (1. + k1 * theta_2 + k2 * theta_4 +
                           k3 * theta_2 * theta_4 + k4 * theta_8);

    Eigen::Vector2d pt_d = P.head<2>() * thetad / rd;
    p << mParameters.mu() * pt_d(0) + mParameters.u0(),
        mParameters.mv() * pt_d(1) + mParameters.v0();
}

/**
 * \brief Project a 3D point to the image plane and calculate Jacobian
 *
 * \param P 3D point coordinates
 * \param p return value, contains the image point coordinates
 */
void EquidistantCamera::spaceToPlane(const Eigen::Vector3d &P,
                                     Eigen::Vector2d &p,
                                     Eigen::Matrix<double, 2, 3> &J) const
{
    double theta = acos(P(2) / P.norm());
    double phi = atan2(P(1), P(0));

    Eigen::Vector2d p_u = r(mParameters.k2(), mParameters.k3(),
                            mParameters.k4(), mParameters.k5(), theta) *
                          Eigen::Vector2d(cos(phi), sin(phi));

    // Apply generalised projection matrix
    p << mParameters.mu() * p_u(0) + mParameters.u0(),
        mParameters.mv() * p_u(1) + mParameters.v0();
}

/**
 * \brief Projects an undistorted 2D point p_u to the image plane
 *
 * \param p_u 2D point coordinates
 * \return image point coordinates
 */
void EquidistantCamera::undistToPlane(const Eigen::Vector2d &p_u,
                                      Eigen::Vector2d &p) const
{
    //    Eigen::Vector2d p_d;
    //
    //    if (m_noDistortion)
    //    {
    //        p_d = p_u;
    //    }
    //    else
    //    {
    //        // Apply distortion
    //        Eigen::Vector2d d_u;
    //        distortion(p_u, d_u);
    //        p_d = p_u + d_u;
    //    }
    //
    //    // Apply generalised projection matrix
    //    p << mParameters.gamma1() * p_d(0) + mParameters.u0(),
    //         mParameters.gamma2() * p_d(1) + mParameters.v0();
}

void EquidistantCamera::initUndistortMap(cv::Mat &map1, cv::Mat &map2,
                                         double fScale) const
{
    cv::Size imageSize(mParameters.imageWidth(), mParameters.imageHeight());

    cv::Mat mapX = cv::Mat::zeros(imageSize, CV_32F);
    cv::Mat mapY = cv::Mat::zeros(imageSize, CV_32F);

    for (int v = 0; v < imageSize.height; ++v) {
        for (int u = 0; u < imageSize.width; ++u) {
            double mx_u = m_inv_K11 / fScale * u + m_inv_K13 / fScale;
            double my_u = m_inv_K22 / fScale * v + m_inv_K23 / fScale;

            double theta, phi;
            backprojectSymmetric(Eigen::Vector2d(mx_u, my_u), theta, phi);

            Eigen::Vector3d P;
            P << sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta);

            Eigen::Vector2d p;
            spaceToPlane(P, p);

            mapX.at<float>(v, u) = p(0);
            mapY.at<float>(v, u) = p(1);
        }
    }

    cv::convertMaps(mapX, mapY, map1, map2, CV_32FC1, false);
}

cv::Mat EquidistantCamera::initUndistortRectifyMap(cv::Mat &map1, cv::Mat &map2,
                                                   float fx, float fy,
                                                   cv::Size imageSize, float cx,
                                                   float cy, cv::Mat rmat) const
{
    if (imageSize == cv::Size(0, 0)) {
        imageSize =
            cv::Size(mParameters.imageWidth(), mParameters.imageHeight());
    }

    cv::Mat mapX = cv::Mat::zeros(imageSize.height, imageSize.width, CV_32F);
    cv::Mat mapY = cv::Mat::zeros(imageSize.height, imageSize.width, CV_32F);

    Eigen::Matrix3f K_rect;

    if (cx == -1.0f && cy == -1.0f) {
        K_rect << fx, 0, imageSize.width / 2, 0, fy, imageSize.height / 2, 0, 0,
            1;
    }
    else {
        K_rect << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    }

    if (fx == -1.0f || fy == -1.0f) {
        K_rect(0, 0) = mParameters.mu();
        K_rect(1, 1) = mParameters.mv();
    }

    Eigen::Matrix3f K_rect_inv = K_rect.inverse();

    Eigen::Matrix3f R, R_inv;
    cv::cv2eigen(rmat, R);
    R_inv = R.inverse();

    for (int v = 0; v < imageSize.height; ++v) {
        for (int u = 0; u < imageSize.width; ++u) {
            Eigen::Vector3f xo;
            xo << u, v, 1;

            Eigen::Vector3f uo = R_inv * K_rect_inv * xo;

            Eigen::Vector2d p;
            spaceToPlane(uo.cast<double>(), p);

            mapX.at<float>(v, u) = p(0);
            mapY.at<float>(v, u) = p(1);
        }
    }

    cv::convertMaps(mapX, mapY, map1, map2, CV_32FC1, false);

    cv::Mat K_rect_cv;
    cv::eigen2cv(K_rect, K_rect_cv);
    return K_rect_cv;
}

int EquidistantCamera::parameterCount() const { return 8; }

const EquidistantCamera::Parameters &EquidistantCamera::getParameters() const
{
    return mParameters;
}

void EquidistantCamera::setParameters(
    const EquidistantCamera::Parameters &parameters)
{
    mParameters = parameters;

    // Inverse camera projection matrix parameters
    m_inv_K11 = 1.0 / mParameters.mu();
    m_inv_K13 = -mParameters.u0() / mParameters.mu();
    m_inv_K22 = 1.0 / mParameters.mv();
    m_inv_K23 = -mParameters.v0() / mParameters.mv();
}

void EquidistantCamera::readParameters(const std::vector<double> &parameterVec)
{
    if (static_cast<int>(parameterVec.size()) != parameterCount()) {
        return;
    }

    Parameters params = getParameters();

    params.k2() = parameterVec.at(0);
    params.k3() = parameterVec.at(1);
    params.k4() = parameterVec.at(2);
    params.k5() = parameterVec.at(3);
    params.mu() = parameterVec.at(4);
    params.mv() = parameterVec.at(5);
    params.u0() = parameterVec.at(6);
    params.v0() = parameterVec.at(7);

    setParameters(params);
}

void EquidistantCamera::writeParameters(std::vector<double> &parameterVec) const
{
    parameterVec.resize(parameterCount());
    parameterVec.at(0) = mParameters.k2();
    parameterVec.at(1) = mParameters.k3();
    parameterVec.at(2) = mParameters.k4();
    parameterVec.at(3) = mParameters.k5();
    parameterVec.at(4) = mParameters.mu();
    parameterVec.at(5) = mParameters.mv();
    parameterVec.at(6) = mParameters.u0();
    parameterVec.at(7) = mParameters.v0();
}

void EquidistantCamera::writeParametersToYamlFile(
    const std::string &filename) const
{
    mParameters.writeToYamlFile(filename);
}

std::string EquidistantCamera::parametersToString() const
{
    std::ostringstream oss;
    oss << mParameters;

    return oss.str();
}

void EquidistantCamera::fitOddPoly(const std::vector<double> &x,
                                   const std::vector<double> &y, int n,
                                   std::vector<double> &coeffs) const
{
    std::vector<int> pows;
    for (int i = 1; i <= n; i += 2) {
        pows.push_back(i);
    }

    Eigen::MatrixXd X(x.size(), pows.size());
    Eigen::MatrixXd Y(y.size(), 1);
    for (size_t i = 0; i < x.size(); ++i) {
        for (size_t j = 0; j < pows.size(); ++j) {
            X(i, j) = pow(x.at(i), pows.at(j));
        }
        Y(i, 0) = y.at(i);
    }

    Eigen::MatrixXd A = (X.transpose() * X).inverse() * X.transpose() * Y;

    coeffs.resize(A.rows());
    for (int i = 0; i < A.rows(); ++i) {
        coeffs.at(i) = A(i, 0);
    }
}

void EquidistantCamera::backprojectSymmetric(const Eigen::Vector2d &p_u,
                                             double &theta, double &phi) const
{
    double p_u_norm = p_u.norm();

    if (p_u_norm < 1e-10) {
        phi = 0.0;
    }
    else {
        phi = std::atan2(p_u(1), p_u(0));
    }

    int npow = 9;
    if (mParameters.k5() == 0.0) {
        npow -= 2;
    }
    if (mParameters.k4() == 0.0) {
        npow -= 2;
    }
    if (mParameters.k3() == 0.0) {
        npow -= 2;
    }
    if (mParameters.k2() == 0.0) {
        npow -= 2;
    }

    Eigen::MatrixXd coeffs(npow + 1, 1);
    coeffs.setZero();
    coeffs(0) = -p_u_norm;
    coeffs(1) = 1.0;

    if (npow >= 3) {
        coeffs(3) = mParameters.k2();
    }
    if (npow >= 5) {
        coeffs(5) = mParameters.k3();
    }
    if (npow >= 7) {
        coeffs(7) = mParameters.k4();
    }
    if (npow >= 9) {
        coeffs(9) = mParameters.k5();
    }

    constexpr double tol = 1e-10;
    if (npow == 1) {
        theta = p_u_norm;
    }
    else {
        // Get eigenvalues of companion matrix corresponding to polynomial.
        // Eigenvalues correspond to roots of polynomial.
        Eigen::MatrixXd A(npow, npow);
        A.setZero();
        A.block(1, 0, npow - 1, npow - 1).setIdentity();
        A.col(npow - 1) = -coeffs.block(0, 0, npow, 1) / coeffs(npow);

        Eigen::EigenSolver<Eigen::MatrixXd> es(A);
        Eigen::MatrixXcd eigval = es.eigenvalues();

        std::vector<double> thetas;
        for (int i = 0; i < eigval.rows(); ++i) {
            if (std::abs(eigval(i).imag()) > tol) {
                continue;
            }

            double t = eigval(i).real();

            if (t < -tol) {
                continue;
            }
            else if (t < 0.0) {
                t = 0.0;
            }

            thetas.push_back(t);
        }

        if (thetas.empty()) {
            theta = p_u_norm;
        }
        else {
            theta = *std::min_element(thetas.begin(), thetas.end());
        }
    }
}

} // namespace camodocal
