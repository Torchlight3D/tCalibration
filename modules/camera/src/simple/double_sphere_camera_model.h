#pragma once

#include "camera_intrinsics.h"
#include <ceres/ceres.h>

namespace tl {

// Brief:
//
// Explanation:
//
// Reference:
class DoubleSphereCameraModel final : public CameraIntrinsics
{
public:
    DoubleSphereCameraModel();

    Type type() const override;

    void setFromMetaData(const CameraMetaData& meta) override;
    CameraMetaData toMetaData() const override;

    // --------------------- Getter and Setter methods ----------------------
    //
    enum IntrinsicsIndex
    {
        Skew = ExtraIndex,
        Xi,    // value range [-1., 1.]
        Alpha, // value range [0., 1.]

        IntrinsicsSize,
    };
    int numParameters() const override;

    void setSkew(double skew);
    double skew() const;

    void setDistortion(double alpha, double xi);
    double alpha() const;
    double xi() const;

    std::vector<int> constantParameterIndices(
        OptimizeIntrinsicsType flags) const override;

    void calibrationMatrix(Eigen::Matrix3d& matrix) const override;

    // --------------------- Point Mapping  ---------------------------
    //
    template <typename T>
    static bool spaceToPixel(const T* intrinsics, const T* point, T* pixel);

    template <typename T>
    static bool pixelToSpace(const T* intrinsics, const T* pixel, T* point);

    template <typename T>
    static bool distort(const T* intrinsics, const T undistort[3],
                        T* distorted);

    template <typename T>
    static bool undistort(const T* intrinsics, const T* distort,
                          T undistorted[3]);

    Eigen::Vector2d spaceToImage(const Eigen::Vector3d& point) const override
    {
        Eigen::Vector2d pixel;
        spaceToPixel(parameters(), point.data(), pixel.data());
        return pixel;
    }

    Eigen::Vector3d imageToSpace(const Eigen::Vector2d& pixel) const override
    {
        Eigen::Vector3d point;
        pixelToSpace(parameters(), pixel.data(), point.data());
        return point;
    }

    Eigen::Vector2d distort(const Eigen::Vector2d& undistort) const override
    {
        return undistort;
    }
    Eigen::Vector2d distort(const Eigen::Vector3d& point3) const
    {
        Eigen::Vector2d distorted;
        distort(parameters(), point3.data(), distorted.data());
        return distorted;
    }

    Eigen::Vector2d undistort(const Eigen::Vector2d& distort) const override
    {
        Eigen::Vector3d undistorted;
        undistort(parameters(), distort.data(), undistorted.data());
        return undistorted.head<2>();
    }

    void print() const override;
};

/// ---------------------- Implementation -----------------------------------
///
template <typename T>
bool DoubleSphereCameraModel::spaceToPixel(const T* intrinsics, const T* point,
                                           T* pixel)
{
    // Not normalize

    // Apply distortion.
    T pt_d[2];
    bool result = DoubleSphereCameraModel::distort(intrinsics, point, pt_d);

    // Apply calibration parameters to transform normalized units into pixels.
    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T fy = y_x * fx;
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];
    const T& skew = intrinsics[Skew];

    pixel[0] = fx * pt_d[0] + skew * pt_d[1] + cx;
    pixel[1] = fy * pt_d[1] + cy;

    return result;
}

template <typename T>
bool DoubleSphereCameraModel::pixelToSpace(const T* intrinsics, const T* pixel,
                                           T* point)
{
    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T fy = fx * y_x;
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];
    const T& skew = intrinsics[Skew];

    // Normalize the y coordinate first.
    T pt_d[2];
    pt_d[1] = (pixel[1] - cy) / fy;
    pt_d[0] = (pixel[0] - cx - pt_d[1] * skew) / fx;

    // undistort
    return DoubleSphereCameraModel::undistort(intrinsics, pt_d, point);
}

template <typename T>
bool DoubleSphereCameraModel::distort(const T* intrinsics, const T* pt_u,
                                      T* pt_d)
{
    const T& alpha = intrinsics[Alpha];
    const T& xi = intrinsics[Xi];

    const T xx = pt_u[0] * pt_u[0];
    const T yy = pt_u[1] * pt_u[1];
    const T zz = pt_u[2] * pt_u[2];

    const T rr = xx + yy;

    const T d1_2 = rr + zz;
    const T d1 = ceres::sqrt(d1_2);

    const T w1 =
        alpha > T(0.5) ? (T(1) - alpha) / alpha : alpha / (T(1) - alpha);
    const T w2 = (w1 + xi) / ceres::sqrt(T(2) * w1 * xi + xi * xi + T(1));

    if (pt_u[2] <= -w2 * d1) {
        return false;
    }

    const T k = xi * d1 + pt_u[2];
    const T kk = k * k;

    const T d2_2 = rr + kk;
    const T d2 = ceres::sqrt(d2_2);

    const T norm = alpha * d2 + (T(1) - alpha) * k;

    pt_d[0] = pt_u[0] / norm;
    pt_d[1] = pt_u[1] / norm;

    return true;
}

template <typename T>
bool DoubleSphereCameraModel::undistort(const T* intrinsics, const T* pt_d,
                                        T* pt_u)
{
    const T& xi = intrinsics[Xi];
    const T& alpha = intrinsics[Alpha];

    const T rr = pt_d[0] * pt_d[0] + pt_d[1] * pt_d[1];

    if (alpha > T(0.5)) {
        if (rr >= T(1) / (T(2) * alpha - T(1))) {
            return false;
        }
    }

    const T xi2_2 = alpha * alpha;
    const T xi1_2 = xi * xi;

    const T sqrt2 = ceres::sqrt(T(1) - (T(2) * alpha - T(1)) * rr);

    const T norm2 = alpha * sqrt2 + T(1) - alpha;

    const T mz = (T(1) - xi2_2 * rr) / norm2;
    const T mz2 = mz * mz;

    const T norm1 = mz2 + rr;
    const T sqrt1 = ceres::sqrt(mz2 + (T(1) - xi1_2) * rr);
    const T k = (mz * xi + sqrt1) / norm1;

    pt_u[0] = k * pt_d[0];
    pt_u[1] = k * pt_d[1];
    pt_u[2] = k * mz - xi;

    return true;
}

} // namespace tl
