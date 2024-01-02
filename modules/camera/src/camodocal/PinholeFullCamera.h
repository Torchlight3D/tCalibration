#pragma once

#include "Camera.h"

#include <string>

#include <ceres/rotation.h>

namespace camodocal {

class PinholeFullCamera final : public Camera
{
public:
    class Parameters : public Camera::Parameters
    {
    public:
        Parameters();
        Parameters(const std::string &cameraName, int w, int h, double k1,
                   double k2, double k3, double k4, double k5, double k6,
                   double p1, double p2, double fx, double fy, double cx,
                   double cy);

        inline static constexpr char kModelTypeName[13]{"PINHOLE_FULL"};
        inline const char *modelTypeName() const override
        {
            return kModelTypeName;
        }

        double &k1();
        double &k2();
        double &k3();
        double &k4();
        double &k5();
        double &k6();
        double &p1();
        double &p2();
        double &fx();
        double &fy();
        double &cx();
        double &cy();

        double xi() const;
        double k1() const;
        double k2() const;
        double k3() const;
        double k4() const;
        double k5() const;
        double k6() const;
        double p1() const;
        double p2() const;
        double fx() const;
        double fy() const;
        double cx() const;
        double cy() const;

        bool read(const cv::FileNode &node) override;
        void write(cv::FileStorage &fs) const override;

        Parameters &operator=(const Parameters &other);
        friend std::ostream &operator<<(std::ostream &out,
                                        const Parameters &params);

    private:
        double m_k1;
        double m_k2;
        double m_k3;
        double m_k4;
        double m_k5;
        double m_k6;
        double m_p1;
        double m_p2;
        double m_fx;
        double m_fy;
        double m_cx;
        double m_cy;
    };

    using Ptr = std::shared_ptr<PinholeFullCamera>;
    using ConstPtr = std::shared_ptr<const PinholeFullCamera>;

    PinholeFullCamera();
    explicit PinholeFullCamera(const Parameters &params);
    PinholeFullCamera(const std::string &cameraName, int imageWidth,
                      int imageHeight, double k1, double k2, double k3,
                      double k4, double k5, double k6, double p1, double p2,
                      double fx, double fy, double cx, double cy);

    Camera::ModelType modelType() const override;
    const std::string &cameraName() const override;
    int imageWidth() const override;
    int imageHeight() const override;

    cv::Point2f getPrinciple() const
    {
        return cv::Point2f(mParameters.cx(), mParameters.cy());
    }

    void estimateIntrinsics(
        const cv::Size &boardSize,
        const std::vector<std::vector<cv::Point3f>> &objectPoints,
        const std::vector<std::vector<cv::Point2f>> &imagePoints) override;

    void setInitIntrinsics(
        const std::vector<std::vector<cv::Point3f>> &objectPoints,
        const std::vector<std::vector<cv::Point2f>> &imagePoints)
    {
        Parameters params = getParameters();

        params.k1() = 0.0;
        params.k2() = 0.0;
        params.k3() = 0.0;
        params.k4() = 0.0;
        params.k5() = 0.0;
        params.k6() = 0.0;
        params.p1() = 0.0;
        params.p2() = 0.0;

        double cx = params.imageWidth() / 2.0;
        double cy = params.imageHeight() / 2.0;
        params.cx() = cx;
        params.cy() = cy;
        params.fx() = 1200;
        params.fy() = 1200;

        setParameters(params);
    }

    // Lift points from the image plane to the sphere
    void liftSphere(const Eigen::Vector2d &p,
                    Eigen::Vector3d &P) const override;

    // Lift points from the image plane to the projective space
    void liftProjective(const Eigen::Vector2d &p,
                        Eigen::Vector3d &P) const override;

    void liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P,
                        float image_scale) const;

    // Projects 3D points to the image plane (Pi function)
    void spaceToPlane(const Eigen::Vector3d &P,
                      Eigen::Vector2d &p) const override;

    void spaceToPlane(const Eigen::Vector3d &P, Eigen::Vector2d &p,
                      float image_scalse) const;

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

    void distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u) const;
    void distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u,
                    Eigen::Matrix2d &J) const;

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

    double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
    bool m_noDistortion;
};

template <typename T>
void PinholeFullCamera::spaceToPlane(const T *const params, const T *const q,
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

    // project 3D object point to the image plane
    T k1 = params[0];
    T k2 = params[1];
    T k3 = params[2];
    T k4 = params[3];
    T k5 = params[4];
    T k6 = params[5];
    T p1 = params[6];
    T p2 = params[7];
    T fx = params[8];
    T fy = params[9];
    T alpha = T(0); // cameraParams.alpha();
    T cx = params[10];
    T cy = params[11];

    // Transform to model plane
    T x = P_c[0] / P_c[2];
    T y = P_c[1] / P_c[2];
    // T z = P_c[2] / P_c[2];

    T r2 = x * x + y * y;
    T r4 = r2 * r2;
    T r6 = r4 * r2;
    T a1 = T(2) * x * y;
    T a2 = r2 + T(2) * x * x;
    T a3 = r2 + T(2) * y * y;
    T cdist = T(1) + k1 * r2 + k2 * r4 + k3 * r6;
    T icdist2 = T(1) / (T(1) + k4 * r2 + k5 * r4 + k6 * r6);
    T xd0 = x * cdist * icdist2 + p1 * a1 + p2 * a2; // + k[8] * r2 + k[9] * r4;
    T yd0 =
        y * cdist * icdist2 + p1 * a3 + p2 * a1; // + k[10] * r2 + k[11] * r4;

    p(0) = xd0 * fx + cx;
    p(1) = yd0 * fy + cy;
}
} // namespace camodocal
