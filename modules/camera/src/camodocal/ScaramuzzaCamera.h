#pragma once

#include "Camera.h"

#include <string>

#include <ceres/rotation.h>

namespace camodocal {

// Reference:
// Scaramuzza Camera (Omnidirectional)
// https://sites.google.com/site/scarabotix/ocamcalib-toolbox
class OCAMCamera final : public Camera
{
public:
    inline static constexpr int kPolySize = 5;
    inline static constexpr int kInvPolySize = 20;
    inline static constexpr int kNumParameters =
        kPolySize + kInvPolySize + 2 /*center*/ + 3 /*affine*/;

    class Parameters : public Camera::Parameters
    {
    public:
        Parameters();

        inline static constexpr char kModelTypeName[11]{"scaramuzza"};
        inline const char *modelTypeName() const override
        {
            return kModelTypeName;
        }

        double &C();
        double &D();
        double &E();

        double &center_x();
        double &center_y();

        double &poly(int idx);
        double &inv_poly(int idx);

        double C() const;
        double D() const;
        double E() const;

        double center_x() const;
        double center_y() const;

        double poly(int idx) const;
        double inv_poly(int idx) const;

        bool read(const cv::FileNode &node) override;
        void write(cv::FileStorage &fs) const override;

        Parameters &operator=(const Parameters &other);
        friend std::ostream &operator<<(std::ostream &out,
                                        const Parameters &params);

    private:
        double m_poly[kPolySize];
        double m_inv_poly[kInvPolySize];
        double m_C, m_D, m_E;
        double m_center_x, m_center_y;
    };

    using Ptr = std::shared_ptr<OCAMCamera>;
    using ConstPtr = std::shared_ptr<const OCAMCamera>;

    OCAMCamera();
    explicit OCAMCamera(const Parameters &params);

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
    // void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
    //                  Eigen::Matrix<double,2,3>& J) const;

    void undistToPlane(const Eigen::Vector2d &p_u,
                       Eigen::Vector2d &p) const override;

    template <typename T>
    static void spaceToPlane(const T *const params, const T *const q,
                             const T *const t, const Eigen::Vector3<T> &P,
                             Eigen::Vector2<T> &p);
    template <typename T>
    static void spaceToSphere(const T *const params, const T *const q,
                              const T *const t, const Eigen::Vector3<T> &P,
                              Eigen::Vector3<T> &P_s);
    template <typename T>
    static void LiftToSphere(const T *const params, const Eigen::Vector2<T> &p,
                             Eigen::Vector3<T> &P);

    template <typename T>
    static void SphereToPlane(const T *const params, const Eigen::Vector3<T> &P,
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
    Parameters mParameters;
    double m_inv_scale;
};

template <typename T>
void OCAMCamera::spaceToPlane(const T *const params, const T *const q,
                              const T *const t, const Eigen::Vector3<T> &P,
                              Eigen::Vector2<T> &p)
{
    T P_c[3];
    {
        T P_w[3];
        P_w[0] = P(0);
        P_w[1] = P(1);
        P_w[2] = P(2);

        // Convert quaternion from Eigen convention (x, y, z, w)
        // to Ceres convention (w, x, y, z)
        T q_ceres[4] = {q[3], q[0], q[1], q[2]};

        ceres::QuaternionRotatePoint(q_ceres, P_w, P_c);

        P_c[0] += t[0];
        P_c[1] += t[1];
        P_c[2] += t[2];
    }

    T c = params[0];
    T d = params[1];
    T e = params[2];
    T xc[2] = {params[3], params[4]};

    // T poly[kPolySize];
    // for (int i = 0; i < kPolySize; i++) {
    //     poly[i] = params[5 + i];
    // }

    T inv_poly[kInvPolySize];
    for (int i = 0; i < kInvPolySize; i++) {
        inv_poly[i] = params[5 + kPolySize + i];
    }

    T norm_sqr = P_c[0] * P_c[0] + P_c[1] * P_c[1];
    T norm = T(0);
    if (norm_sqr > T(0)) {
        norm = sqrt(norm_sqr);
    }

    T theta = atan2(-P_c[2], norm);
    T rho = T(0);
    T theta_i = T(1);

    for (int i = 0; i < kInvPolySize; i++) {
        rho += theta_i * inv_poly[i];
        theta_i *= theta;
    }

    T invNorm = T(1) / norm;
    T xn[2] = {P_c[0] * invNorm * rho, P_c[1] * invNorm * rho};

    p(0) = xn[0] * c + xn[1] * d + xc[0];
    p(1) = xn[0] * e + xn[1] + xc[1];
}

template <typename T>
void OCAMCamera::spaceToSphere(const T *const params, const T *const q,
                               const T *const t, const Eigen::Vector3<T> &P,
                               Eigen::Vector3<T> &P_s)
{
    T P_c[3];
    {
        T P_w[3];
        P_w[0] = P(0);
        P_w[1] = P(1);
        P_w[2] = P(2);

        // Convert quaternion from Eigen convention (x, y, z, w)
        // to Ceres convention (w, x, y, z)
        T q_ceres[4] = {q[3], q[0], q[1], q[2]};

        ceres::QuaternionRotatePoint(q_ceres, P_w, P_c);

        P_c[0] += t[0];
        P_c[1] += t[1];
        P_c[2] += t[2];
    }

    // T poly[kPolySize];
    // for (int i = 0; i < kPolySize; i++) {
    //     poly[i] = params[5 + i];
    // }

    T norm_sqr = P_c[0] * P_c[0] + P_c[1] * P_c[1] + P_c[2] * P_c[2];
    T norm = T(0);
    if (norm_sqr > T(0)) {
        norm = sqrt(norm_sqr);
    }

    P_s(0) = P_c[0] / norm;
    P_s(1) = P_c[1] / norm;
    P_s(2) = P_c[2] / norm;
}

template <typename T>
void OCAMCamera::LiftToSphere(const T *const params, const Eigen::Vector2<T> &p,
                              Eigen::Vector3<T> &P)
{
    T c = params[0];
    T d = params[1];
    T e = params[2];
    T cc[2] = {params[3], params[4]};

    T poly[kPolySize];
    for (int i = 0; i < kPolySize; i++) {
        poly[i] = params[5 + i];
    }

    // Relative to Center
    T p_2d[2];
    p_2d[0] = p(0);
    p_2d[1] = p(1);

    T xc[2] = {p_2d[0] - cc[0], p_2d[1] - cc[1]};

    T inv_scale = T(1) / (c - d * e);

    // Affine Transformation
    T xc_a[2];
    xc_a[0] = inv_scale * (xc[0] - d * xc[1]);
    xc_a[1] = inv_scale * (-e * xc[0] + c * xc[1]);

    T norm_sqr = xc_a[0] * xc_a[0] + xc_a[1] * xc_a[1];
    T phi = sqrt(norm_sqr);
    T phi_i = T(1);
    T z = T(0);

    for (int i = 0; i < kPolySize; i++) {
        if (i != 1) {
            z += phi_i * poly[i];
        }
        phi_i *= phi;
    }

    T p_3d[3];
    p_3d[0] = xc[0];
    p_3d[1] = xc[1];
    p_3d[2] = -z;

    T p_3d_norm_sqr = p_3d[0] * p_3d[0] + p_3d[1] * p_3d[1] + p_3d[2] * p_3d[2];
    T p_3d_norm = sqrt(p_3d_norm_sqr);

    P << p_3d[0] / p_3d_norm, p_3d[1] / p_3d_norm, p_3d[2] / p_3d_norm;
}

template <typename T>
void OCAMCamera::SphereToPlane(const T *const params,
                               const Eigen::Vector3<T> &P, Eigen::Vector2<T> &p)
{
    T P_c[3];
    P_c[0] = P(0);
    P_c[1] = P(1);
    P_c[2] = P(2);

    T c = params[0];
    T d = params[1];
    T e = params[2];
    T xc[2] = {params[3], params[4]};

    T inv_poly[kInvPolySize];
    for (int i = 0; i < kInvPolySize; i++) {
        inv_poly[i] = params[5 + kPolySize + i];
    }

    T norm_sqr = P_c[0] * P_c[0] + P_c[1] * P_c[1];
    T norm = T(0);
    if (norm_sqr > T(0)) {
        norm = std::sqrt(norm_sqr);
    }

    T theta = atan2(-P_c[2], norm);
    T rho = T(0);
    T theta_i = T(1);

    for (int i = 0; i < kInvPolySize; i++) {
        rho += theta_i * inv_poly[i];
        theta_i *= theta;
    }

    T invNorm = T(1) / norm;
    T xn[2] = {P_c[0] * invNorm * rho, P_c[1] * invNorm * rho};

    p(0) = xn[0] * c + xn[1] * d + xc[0];
    p(1) = xn[0] * e + xn[1] + xc[1];
}

} // namespace camodocal
