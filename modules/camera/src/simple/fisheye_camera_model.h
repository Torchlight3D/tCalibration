#pragma once

#include "camera_intrinsics.h"
#include <ceres/ceres.h>

namespace tl {

// Brief:
// FisheyeCameraModel
//
// Explanation:
//
// Reference:
class FisheyeCameraModel final : public CameraIntrinsics
{
public:
    FisheyeCameraModel();

    Type type() const override;

    void setFromMetaData(const CameraMetaData& meta) override;
    CameraMetaData toMetaData() const override;

    // ----------------------- Parameters Access ----------------------
    //
    enum IntrinsicsIndex
    {
        Skew = ExtraIndex,
        K1,
        K2,
        K3,
        K4,

        IntrinsicsSize,
    };

    void setSkew(double skew);
    double skew() const;

    void setRadialDistortion(double k1, double k2, double k3, double k4);
    double radialDistortion1() const;
    double radialDistortion2() const;
    double radialDistortion3() const;
    double radialDistortion4() const;
    inline auto k1() const { return radialDistortion1(); }
    inline auto k2() const { return radialDistortion2(); }
    inline auto k3() const { return radialDistortion3(); }
    inline auto k4() const { return radialDistortion4(); }

    int numParameters() const override;

    std::vector<int> constantParameterIndices(
        OptimizeIntrinsicsType flags) const override;

    void calibrationMatrix(Eigen::Matrix3d& matrix) const override;

    // --------------------- Point Mapping --------------------------------
    //
    template <typename T>
    static bool spaceToPixel(const T* intrinsics, const T* point, T* pixel);

    template <typename T>
    static bool pixelToSpace(const T* intrinsics, const T* pixel, T* point);

    template <typename T>
    static bool distort(const T* intrinsics, const T* undistort, T* distorted);

    template <typename T>
    static bool undistort(const T* intrinsics, const T* distort,
                          T* undistorted);

    template <typename T>
    static bool calcDistortion(const T* intrinsics, const T* undistort,
                               T* theta, T* radialDistortion);

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

    // FIXME: The input here should be Vector3
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
        Eigen::Vector2d undistorted;
        undistort(parameters(), distort.data(), undistorted.data());
        return undistorted;
    }

    void print() const override;
};

/// -------------------------- Implementation -------------------------------
///
template <typename T>
bool FisheyeCameraModel::spaceToPixel(const T* intrinsics, const T* point,
                                      T* pixel)
{
    // Apply radial distortion. Note that we pass in the entire 3D point instead
    // of the projection onto a plane or sphere.
    T pt_d[2];
    FisheyeCameraModel::distort(intrinsics, point, pt_d);

    // Apply calibration parameters to transform normalized units into pixels.
    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T fy = fx * y_x;
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];
    const T& skew = intrinsics[Skew];

    pixel[0] = fx * pt_d[0] + skew * pt_d[1] + cx;
    pixel[1] = fy * pt_d[1] + cy;

    return true;
}

template <typename T>
bool FisheyeCameraModel::pixelToSpace(const T* intrinsics, const T* px, T* pt)
{
    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T fy = fx * y_x;
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];
    const T& skew = intrinsics[Skew];

    // Normalize the y coordinate first.
    T px_d[2];
    px_d[1] = (px[1] - cy) / fy;
    px_d[0] = (px[0] - cx - px_d[1] * skew) / fx;

    // Undo the radial distortion.
    FisheyeCameraModel::undistort(intrinsics, px_d, pt);
    pt[2] = T(1);

    return true;
}

// For fisheye distortion, we use the angle of the undistorted point with
// respect to the optical axis to measure the magnitude of distortion. To make
// sure that the computation is stable, we pass in the 3-dimensional undistorted
// point and compute angle theta using the more stable atan2 so that we do not
// have to perform the (potentially unstable) perspective divide by z.
template <typename T>
bool FisheyeCameraModel::distort(const T* intrinsics, const T* pt_u, T* pt_d)
{
    T thetad;
    T rd;
    bool distortion = calcDistortion(intrinsics, pt_u, &thetad, &rd);
    if (!distortion) {
        pt_d[0] = pt_u[0];
        pt_d[1] = pt_u[1];
        return true;
    }

    pt_d[0] = thetad * pt_u[0] / rd;
    pt_d[1] = thetad * pt_u[1] / rd;

    if (pt_u[2] < T(0)) {
        pt_d[0] = -pt_d[0];
        pt_d[1] = -pt_d[1];
    }

    return true;
}

template <typename T>
bool FisheyeCameraModel::undistort(const T* intrinsics, const T* pt_d, T* pt_u)
{
    constexpr int kIterations{100};
    const T kUndistortionEpsilon = T(1e-10);

    T prev_pt_u[2];
    pt_u[0] = pt_d[0];
    pt_u[1] = pt_d[1];
    for (int i{0}; i < kIterations; ++i) {
        prev_pt_u[0] = pt_u[0];
        prev_pt_u[1] = pt_u[1];

        const T pt_u_norm[3]{pt_u[0], pt_u[1], T(1.)};
        T thetad, rd;
        bool distortion = calcDistortion(intrinsics, pt_u_norm, &thetad, &rd);
        if (!distortion) {
            pt_u[0] = pt_d[0];
            pt_u[1] = pt_d[1];
            return true;
        }

        // We know that the distorted point = theta_d / r * undistorted point,
        // so we can solve for a better estimate of the undistorted point by
        // taking the inverse of this equation:
        //   undistorted_point = r * distorted_point / theta_d.
        pt_u[0] = rd * pt_d[0] / thetad;
        pt_u[1] = rd * pt_d[1] / thetad;

        if (ceres::abs(pt_u[0] - prev_pt_u[0]) < kUndistortionEpsilon &&
            ceres::abs(pt_u[1] - prev_pt_u[1]) < kUndistortionEpsilon) {
            break;
        }
    }
    return true;
}

template <typename T>
bool FisheyeCameraModel::calcDistortion(const T* intrinsics, const T* pt_u,
                                        T* thetad, T* rd)
{
    const T& k1 = intrinsics[K1];
    const T& k2 = intrinsics[K2];
    const T& k3 = intrinsics[K3];
    const T& k4 = intrinsics[K4];

    const T rr = pt_u[0] * pt_u[0] + pt_u[1] * pt_u[1];

    // If the radius of the undistorted is too small then the divide by r below
    // is unstable. In this case, the point is very close to the center of
    // distortion and so we can assume there is no distortion.
    static const T kVerySmallNumber = T(1e-8);
    if (rr < kVerySmallNumber) {
        return false;
    }

    *rd = ceres::sqrt(rr);
    // Using atan2 should be more stable than dividing by the denominator for
    // small z-values (i.e. viewing angles approaching 180 deg FOV).
    const T theta = ceres::atan2(*rd, ceres::abs(pt_u[2]));
    const T theta_2 = theta * theta;
    const T theta_4 = theta_2 * theta_2;
    const T theta_8 = theta_4 * theta_4;
    *thetad = theta * (T(1) + k1 * theta_2 + k2 * theta_4 +
                       k3 * theta_2 * theta_4 + k4 * theta_8);

    return true;
}

} // namespace tl
