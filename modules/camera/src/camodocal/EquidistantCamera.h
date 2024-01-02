#pragma once

#include "Camera.h"

#include <string>

#include <ceres/rotation.h>

namespace camodocal {

/**
 * J. Kannala, and S. Brandt, A Generic Camera Model and Calibration Method
 * for Conventional, Wide-Angle, and Fish-Eye Lenses, PAMI 2006
 */

class EquidistantCamera final : public Camera
{
public:
    class Parameters : public Camera::Parameters
    {
    public:
        Parameters();
        Parameters(const std::string &cameraName, int w, int h, double k2,
                   double k3, double k4, double k5, double mu, double mv,
                   double u0, double v0);

        inline static constexpr char kModelTypeName[15]{"KANNALA_BRANDT"};
        inline const char *modelTypeName() const override
        {
            return kModelTypeName;
        }

        double &k2();
        double &k3();
        double &k4();
        double &k5();
        double &mu();
        double &mv();
        double &u0();
        double &v0();

        double k2() const;
        double k3() const;
        double k4() const;
        double k5() const;
        double mu() const;
        double mv() const;
        double u0() const;
        double v0() const;

        bool read(const cv::FileNode &node) override;
        void write(cv::FileStorage &fs) const override;

        Parameters &operator=(const Parameters &other);
        friend std::ostream &operator<<(std::ostream &out,
                                        const Parameters &params);

    private:
        // projection
        double m_k2;
        double m_k3;
        double m_k4;
        double m_k5;

        double m_mu;
        double m_mv;
        double m_u0;
        double m_v0;
    };

    using Ptr = std::shared_ptr<EquidistantCamera>;
    using ConstPtr = std::shared_ptr<const EquidistantCamera>;

    EquidistantCamera();
    explicit EquidistantCamera(const Parameters &params);
    EquidistantCamera(const std::string &cameraName, int imageWidth,
                      int imageHeight, double k2, double k3, double k4,
                      double k5, double mu, double mv, double u0, double v0);

    Camera::ModelType modelType() const override;
    const std::string &cameraName() const override;
    int imageWidth() const override;
    int imageHeight() const override;

    void estimateIntrinsics(
        const cv::Size &boardSize,
        const std::vector<std::vector<cv::Point3f>> &objectPoints,
        const std::vector<std::vector<cv::Point2f>> &imagePoints) override;

    // Lift points from the image plane to the sphere
    void liftSphere(const Eigen::Vector2d &p,
                    Eigen::Vector3d &P) const override;

    // Lift points from the image plane to the projective space
    void liftProjective(const Eigen::Vector2d &p,
                        Eigen::Vector3d &P) const override;

    // Projects 3D points to the image plane (Pi function)
    void spaceToPlane(const Eigen::Vector3d &P,
                      Eigen::Vector2d &p) const override;

    // Projects 3D points to the image plane (Pi function)
    // and calculates jacobian
    void spaceToPlane(const Eigen::Vector3d &P, Eigen::Vector2d &p,
                      Eigen::Matrix<double, 2, 3> &J) const;

    void undistToPlane(const Eigen::Vector2d &p_u,
                       Eigen::Vector2d &p) const override;

    template <typename T>
    static void spaceToPlane(const T *const params, const T *const q,
                             const T *const t, const Eigen::Vector3<T> &P,
                             Eigen::Vector2<T> &p);

    void initUndistortMap(cv::Mat &map1, cv::Mat &map2,
                          double fScale = 1.0) const;
    cv::Mat initUndistortRectifyMap(
        cv::Mat &map1, cv::Mat &map2, float fx = -1.0f, float fy = -1.0f,
        cv::Size imageSize = cv::Size(0, 0), float cx = -1.0f, float cy = -1.0f,
        cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const override;

    int parameterCount() const override;

    const Parameters &getParameters() const;
    void setParameters(const Parameters &parameters);

    void readParameters(const std::vector<double> &parameters) override;
    void writeParameters(std::vector<double> &parameters) const override;

    void writeParametersToYamlFile(const std::string &filename) const override;

    std::string parametersToString() const override;

private:
    template <typename T>
    static constexpr T r(T k2, T k3, T k4, T k5, T theta);

    void fitOddPoly(const std::vector<double> &x, const std::vector<double> &y,
                    int n, std::vector<double> &coeffs) const;

    void backprojectSymmetric(const Eigen::Vector2d &p_u, double &theta,
                              double &phi) const;

private:
    Parameters mParameters;
    double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
};

template <typename T>
constexpr T EquidistantCamera::r(T k2, T k3, T k4, T k5, T theta)
{
    // k1 = 1
    return theta + k2 * theta * theta * theta +
           k3 * theta * theta * theta * theta * theta +
           k4 * theta * theta * theta * theta * theta * theta * theta +
           k5 * theta * theta * theta * theta * theta * theta * theta * theta *
               theta;
}

template <typename T>
void EquidistantCamera::spaceToPlane(const T *const params, const T *const q,
                                     const T *const t,
                                     const Eigen::Vector3<T> &P,
                                     Eigen::Vector2<T> &p)
{
    T P_w[3];
    P_w[0] = P(0);
    P_w[1] = P(1);
    P_w[2] = P(2);

    // Convert quaternion from Eigen convention (x, y, z, w)
    // to Ceres convention (w, x, y, z)
    T q_ceres[4] = {q[3], q[0], q[1], q[2]};

    T P_c[3];
    ceres::QuaternionRotatePoint(q_ceres, P_w, P_c);

    P_c[0] += t[0];
    P_c[1] += t[1];
    P_c[2] += t[2];

    // project 3D object point to the image plane;
    T k2 = params[0];
    T k3 = params[1];
    T k4 = params[2];
    T k5 = params[3];
    T mu = params[4];
    T mv = params[5];
    T u0 = params[6];
    T v0 = params[7];

    T len = std::sqrt(P_c[0] * P_c[0] + P_c[1] * P_c[1] + P_c[2] * P_c[2]);
    T theta = std::acos(P_c[2] / len);
    T phi = std::atan2(P_c[1], P_c[0]);

    Eigen::Vector2<T> p_u = r(k2, k3, k4, k5, theta) *
                            Eigen::Vector2<T>(std::cos(phi), std::sin(phi));

    p(0) = mu * p_u(0) + u0;
    p(1) = mv * p_u(1) + v0;
}

} // namespace camodocal
