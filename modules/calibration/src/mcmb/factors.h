#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>

namespace tl::mcmb {

// Intrinsic and board pose refinement
struct ReprojectionError
{
    double u, v;
    double x, y, z;
    int distortion_type;

    ReprojectionError(double u, double v, double x, double y, double z,
                      int type)
        : u(u), v(v), x(x), y(y), z(z), distortion_type(type)
    {
    }

    // Extrinsics: angle axis + translation
    template <typename T>
    bool operator()(const T *const extrinsics, const T *const intrinsics,
                    T *residuals) const
    {
        T p[3];
        const T point[3] = {T(x), T(y), T(z)};
        ceres::AngleAxisRotatePoint(extrinsics, point, p);

        p[0] += extrinsics[3];
        p[1] += extrinsics[4];
        p[2] += extrinsics[5];

        p[0] /= p[2];
        p[1] /= p[2];

        const auto &fx = intrinsics[0];
        const auto &fy = intrinsics[1];
        const auto &cx = intrinsics[2];
        const auto &cy = intrinsics[3];

        if (distortion_type == 0) // perspective brown
        {
            // Distorsion
            const auto &k1 = intrinsics[4];
            const auto &k2 = intrinsics[5];
            const auto &k3 = intrinsics[8];
            const auto &p1 = intrinsics[6];
            const auto &p2 = intrinsics[7];

            T r2 = p[0] * p[0] + p[1] * p[1];
            T r4 = r2 * r2;
            T r6 = r4 * r2;
            T thetad = (T(1) + k1 * r2 + k2 * r4 + k3 * r6);

            T xd = p[0] * thetad + T(2) * p1 * p[0] * p[1] +
                   p2 * (r2 + T(2) * p[0] * p[0]);
            T yd = p[1] * thetad + p1 * (r2 + T(2) * (p[1] * p[1])) +
                   T(2) * p2 * p[0] * p[1];

            T up = fx * xd + cx;
            T vp = fy * yd + cy;

            residuals[0] = up - T(u);
            residuals[1] = vp - T(v);
        }

        if (distortion_type == 1) // fisheye
        {
            // Distorsion
            const auto &k1 = intrinsics[4];
            const auto &k2 = intrinsics[5];
            const auto &k3 = intrinsics[6];
            const auto &k4 = intrinsics[7];

            T r2 = p[0] * p[0] + p[1] * p[1];
            T r = sqrt(r2);
            T theta = atan(r);
            T theta2 = theta * theta;
            auto theta3 = theta2 * theta;
            auto theta4 = theta2 * theta2;
            auto theta5 = theta4 * theta;
            T theta6 = theta3 * theta3;
            auto theta7 = theta6 * theta;
            auto theta8 = theta4 * theta4;
            auto theta9 = theta8 * theta;
            T theta_d =
                theta + k1 * theta3 + k2 * theta5 + k3 * theta7 + k4 * theta9;
            T inv_r = r > T(1e-8) ? T(1.0) / r : T(1);
            T cdist = r > T(1e-8) ? theta_d * inv_r : T(1);
            T xd = p[0] * cdist;
            T yd = p[1] * cdist;

            T up = fx * xd + cx;
            T vp = fy * yd + cy;

            residuals[0] = up - T(u);
            residuals[1] = vp - T(v);
        }

        return true;
    }

    static ceres::CostFunction *create(double u, double v, double x, double y,
                                       double z, int type)
    {
        return new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 9>(
            new ReprojectionError(u, v, x, y, z, type));
    }
};

// 3D object refinement (board pose + object pose)
struct ReprojectionError_3DObjRef
{
    double u, v;
    double x, y, z;
    double fx, fy;
    double cx, cy;
    double k1, k2, k3;
    double p1, p2;
    int distortion_type;
    bool refine_board;

    ReprojectionError_3DObjRef(double u, double v, double x, double y, double z,
                               double focal_x, double focal_y, double u0,
                               double v0, double k1, double k2, double k3,
                               double p1, double p2, bool refine_board,
                               int type)
        : u(u),
          v(v),
          x(x),
          y(y),
          z(z),
          fx(focal_x),
          fy(focal_y),
          cx(u0),
          cy(v0),
          k1(k1),
          k2(k2),
          k3(k3),
          p1(p1),
          p2(p2),
          distortion_type(type),
          refine_board(refine_board)
    {
    }

    template <typename T>
    bool operator()(const T *const extrinsics, const T *const boardPose,
                    T *residuals) const
    {
        // apply transformation to the board
        T pboard[3];
        const T point[3] = {T(x), T(y), T(z)};
        if (refine_board) {
            ceres::AngleAxisRotatePoint(boardPose, point, pboard);
            pboard[0] += boardPose[3];
            pboard[1] += boardPose[4];
            pboard[2] += boardPose[5];
        }
        else {
            pboard[0] = point[0];
            pboard[1] = point[1];
            pboard[2] = point[2];
        }

        T p[3];
        ceres::AngleAxisRotatePoint(extrinsics, pboard, p);

        p[0] += extrinsics[3];
        p[1] += extrinsics[4];
        p[2] += extrinsics[5];

        p[0] /= p[2];
        p[1] /= p[2];

        if (distortion_type == 0) // Pinhole
        {
            // apply distorsion
            T r2 = p[0] * p[0] + p[1] * p[1];
            T r4 = r2 * r2;
            T r6 = r4 * r2;
            T thetad = (T(1) + k1 * r2 + k2 * r4 + k3 * r6);

            T xd = p[0] * thetad + T(2) * p1 * p[0] * p[1] +
                   p2 * (r2 + T(2) * p[0] * p[0]);
            T yd = p[1] * thetad + p1 * (r2 + T(2) * (p[1] * p[1])) +
                   T(2) * p2 * p[0] * p[1];

            // Project on the image plane
            T up = T(fx) * xd + T(cx);
            T vp = T(fy) * yd + T(cy);

            residuals[0] = up - T(u);
            residuals[1] = vp - T(v);
        }

        if (distortion_type == 1) // fisheye
        {
            T r2 = p[0] * p[0] + p[1] * p[1];

            T r = sqrt(r2);
            T theta = atan(r);
            T theta2 = theta * theta, theta3 = theta2 * theta,
              theta4 = theta2 * theta2, theta5 = theta4 * theta;
            T theta6 = theta3 * theta3, theta7 = theta6 * theta,
              theta8 = theta4 * theta4, theta9 = theta8 * theta;
            T theta_d =
                theta + k1 * theta3 + k2 * theta5 + p1 * theta7 + p2 * theta9;
            T inv_r = r > T(1e-8) ? T(1.0) / r : T(1);
            T cdist = r > T(1e-8) ? theta_d * inv_r : T(1);

            T xd = p[0] * cdist;
            T yd = p[1] * cdist;

            // Project on the image plane
            T up = T(fx) * xd + T(cx);
            T vp = T(fy) * yd + T(cy);

            residuals[0] = up - T(u);
            residuals[1] = vp - T(v);
        }

        return true;
    }

    static ceres::CostFunction *create(double u, double v, double x, double y,
                                       double z, double focal_x, double focal_y,
                                       double u0, double v0, double k1,
                                       double k2, double k3, double p1,
                                       double p2, bool refine_board, int type)
    {
        return new ceres::AutoDiffCostFunction<ReprojectionError_3DObjRef, 2, 6,
                                               6>(
            new ReprojectionError_3DObjRef(u, v, x, y, z, focal_x, focal_y, u0,
                                           v0, k1, k2, k3, p1, p2, refine_board,
                                           type));
    }
};

// Refine camera group (3D object pose and relative camera pose)
// 3D object refinement (board pose + object pose)
struct ReprojectionError_CameraGroupRef
{
    double u, v;
    double x, y, z;
    double focal_x, focal_y;
    double u0, v0;
    double k1, k2, k3;
    double p1, p2;
    bool refine_camera;
    int distortion_type;

    ReprojectionError_CameraGroupRef(double u, double v, double x, double y,
                                     double z, double focal_x, double focal_y,
                                     double u0, double v0, double k1, double k2,
                                     double k3, double p1, double p2,
                                     bool refine_camera, int type)
        : u(u),
          v(v),
          x(x),
          y(y),
          z(z),
          focal_x(focal_x),
          focal_y(focal_y),
          u0(u0),
          v0(v0),
          k1(k1),
          k2(k2),
          k3(k3),
          p1(p1),
          p2(p2),
          refine_camera(refine_camera),
          distortion_type(type)
    {
    }

    template <typename T>
    bool operator()(const T *const cameraPose, const T *const objectPose,
                    T *residuals) const
    {
        // 1. apply transformation to the object (to expressed in the current
        // camera)
        T pobj[3];
        const T point[3] = {T(x), T(y), T(z)};
        ceres::AngleAxisRotatePoint(objectPose, point, pobj);
        pobj[0] += objectPose[3];
        pobj[1] += objectPose[4];
        pobj[2] += objectPose[5];

        // 2. Refine the camera if it is not the referential
        if (refine_camera) {
            T pobj_refine[3];
            ceres::AngleAxisRotatePoint(cameraPose, pobj, pobj_refine);
            std::copy_n(pobj_refine, 3, pobj);
            pobj[0] += cameraPose[3];
            pobj[1] += cameraPose[4];
            pobj[2] += cameraPose[5];
        }

        // Normalization on the camera plane
        pobj[0] /= pobj[2];
        pobj[1] /= pobj[2];

        if (distortion_type == 0) // perspective brown
        {
            // apply distorsion
            T r2 = pobj[0] * pobj[0] + pobj[1] * pobj[1];
            T r4 = r2 * r2;
            T r6 = r4 * r2;
            T r_coeff = (T(1) + k1 * r2 + k2 * r4 + k3 * r6);
            T xd = pobj[0] * r_coeff + T(2) * p1 * pobj[0] * pobj[1] +
                   p2 * (r2 + T(2) * pobj[0] * pobj[0]);
            T yd = pobj[1] * r_coeff + p1 * (r2 + T(2) * (pobj[1] * pobj[1])) +
                   T(2) * p2 * pobj[0] * pobj[1];

            // Project on the image plane
            T up = T(focal_x) * xd + T(u0);
            T vp = T(focal_y) * yd + T(v0);

            residuals[0] = up - T(u);
            residuals[1] = vp - T(v);
        }

        if (distortion_type == 1) // fisheye
        {
            T r2 = pobj[0] * pobj[0] + pobj[1] * pobj[1];

            T r = sqrt(r2);
            T theta = atan(r);
            T theta2 = theta * theta, theta3 = theta2 * theta,
              theta4 = theta2 * theta2, theta5 = theta4 * theta;
            T theta6 = theta3 * theta3, theta7 = theta6 * theta,
              theta8 = theta4 * theta4, theta9 = theta8 * theta;
            T theta_d =
                theta + k1 * theta3 + k2 * theta5 + p1 * theta7 + p2 * theta9;
            T inv_r = r > T(1e-8) ? T(1) / r : T(1);
            T cdist = r > T(1e-8) ? theta_d * inv_r : T(1);
            T xd = pobj[0] * cdist;
            T yd = pobj[1] * cdist;

            // Project on the image plane
            T up = T(focal_x) * xd + T(u0);
            T vp = T(focal_y) * yd + T(v0);

            residuals[0] = up - T(u);
            residuals[1] = vp - T(v);
        }

        return true;
    }

    static ceres::CostFunction *create(double u, double v, double x, double y,
                                       double z, double fx, double fy,
                                       double u0, double v0, double k1,
                                       double k2, double k3, double p1,
                                       double p2, bool refine_camera, int type)
    {
        return new ceres::AutoDiffCostFunction<ReprojectionError_CameraGroupRef,
                                               2, 6, 6>(
            new ReprojectionError_CameraGroupRef(u, v, x, y, z, fx, fy, u0, v0,
                                                 k1, k2, k3, p1, p2,
                                                 refine_camera, type));
    }
};

// Refine camera group (3D object pose + relative camera pose + Board pose)
struct ReprojectionError_CameraGroupAndObjectRef
{
    double u, v;
    double x, y, z;
    double focal_x, focal_y;
    double u0, v0;
    double k1, k2, k3;
    double p1, p2;
    bool refine_camera;
    bool refine_board;
    int distortion_type;

    ReprojectionError_CameraGroupAndObjectRef(
        double u, double v, double x, double y, double z, double focal_x,
        double focal_y, double u0, double v0, double k1, double k2, double k3,
        double p1, double p2, bool refine_camera, bool refine_board, int type)
        : u(u),
          v(v),
          x(x),
          y(y),
          z(z),
          focal_x(focal_x),
          focal_y(focal_y),
          u0(u0),
          v0(v0),
          k1(k1),
          k2(k2),
          k3(k3),
          p1(p1),
          p2(p2),
          refine_camera(refine_camera),
          refine_board(refine_board),
          distortion_type(type)
    {
    }

    template <typename T>
    bool operator()(const T *const cameraPose, const T *const objectPose,
                    const T *const boardPose, T *residuals) const
    {
        // 1. Apply the board transformation in the object
        T point[3] = {T(x), T(y), T(z)};
        if (refine_board) {
            T point_refine[3];
            ceres::AngleAxisRotatePoint(boardPose, point, point_refine);
            std::copy_n(point_refine, 3, point);
            point[0] += boardPose[3];
            point[1] += boardPose[4];
            point[2] += boardPose[5];
        }

        // 2. apply transformation to the object (to expressed in the current
        // camera)
        T pobj[3];
        ceres::AngleAxisRotatePoint(objectPose, point, pobj);
        pobj[0] += objectPose[3];
        pobj[1] += objectPose[4];
        pobj[2] += objectPose[5];

        // 3. Refine the camera if it is not the referential
        if (refine_camera) {
            T pobj_refine[3];
            ceres::AngleAxisRotatePoint(cameraPose, pobj, pobj_refine);
            std::copy_n(pobj_refine, 3, pobj);
            pobj[0] += cameraPose[3];
            pobj[1] += cameraPose[4];
            pobj[2] += cameraPose[5];
        }

        // Normalization on the camera plane
        pobj[0] /= pobj[2];
        pobj[1] /= pobj[2];

        if (distortion_type == 0) // perspective brown
        {
            T r2 = pobj[0] * pobj[0] + pobj[1] * pobj[1];
            T r4 = r2 * r2;
            T r6 = r4 * r2;
            T r_coeff = (T(1) + k1 * r2 + k2 * r4 + k3 * r6);
            T xd = pobj[0] * r_coeff + T(2) * p1 * pobj[0] * pobj[1] +
                   p2 * (r2 + T(2) * pobj[0] * pobj[0]);
            T yd = pobj[1] * r_coeff + p1 * (r2 + T(2) * (pobj[1] * pobj[1])) +
                   T(2) * p2 * pobj[0] * pobj[1];

            T up = T(focal_x) * xd + T(u0);
            T vp = T(focal_y) * yd + T(v0);

            residuals[0] = up - T(u);
            residuals[1] = vp - T(v);
        }

        if (distortion_type == 1) // fisheye
        {
            T r2 = pobj[0] * pobj[0] + pobj[1] * pobj[1];
            T r = sqrt(r2);
            T theta = atan(r);
            T theta2 = theta * theta;
            auto theta3 = theta2 * theta;
            auto theta4 = theta2 * theta2;
            auto theta5 = theta4 * theta;
            T theta6 = theta3 * theta3;
            auto theta7 = theta6 * theta;
            auto theta8 = theta4 * theta4;
            auto theta9 = theta8 * theta;
            T theta_d =
                theta + k1 * theta3 + k2 * theta5 + p1 * theta7 + p2 * theta9;
            T inv_r = r > T(1e-8) ? T(1.0) / r : T(1);
            T cdist = r > T(1e-8) ? theta_d * inv_r : T(1);
            T xd = pobj[0] * cdist;
            T yd = pobj[1] * cdist;

            T up = T(focal_x) * xd + T(u0);
            T vp = T(focal_y) * yd + T(v0);

            residuals[0] = up - T(u);
            residuals[1] = vp - T(v);
        }

        return true;
    }

    static ceres::CostFunction *create(double u, double v, double x, double y,
                                       double z, double focal_x, double focal_y,
                                       double u0, double v0, double k1,
                                       double k2, double k3, double p1,
                                       double p2, bool refine_camera,
                                       bool refine_board, int type)
    {
        return new ceres::AutoDiffCostFunction<
            ReprojectionError_CameraGroupAndObjectRef, 2, 6, 6, 6>(
            new ReprojectionError_CameraGroupAndObjectRef(
                u, v, x, y, z, focal_x, focal_y, u0, v0, k1, k2, k3, p1, p2,
                refine_camera, refine_board, type));
    }
};

// Refine camera group (3D object pose + relative camera pose + Board pose)
struct ReprojectionError_CameraGroupAndObjectRefAndIntrinsics
{
    double u, v;
    double x, y, z;
    int distortion_type;
    bool refine_camera;
    bool refine_board;

    ReprojectionError_CameraGroupAndObjectRefAndIntrinsics(
        double u, double v, double x, double y, double z, bool refine_camera,
        bool refine_board, int type)
        : u(u),
          v(v),
          x(x),
          y(y),
          z(z),
          refine_camera(refine_camera),
          refine_board(refine_board),
          distortion_type(type)
    {
    }

    template <typename T>
    bool operator()(const T *const cameraPose, const T *const objectPose,
                    const T *const boardPose, const T *const intrinsics,
                    T *residuals) const
    {
        // Alias
        const auto &fx = intrinsics[0];
        const auto &fy = intrinsics[1];
        const auto &cx = intrinsics[2];
        const auto &cy = intrinsics[3];
        const auto &k1 = intrinsics[4];
        const auto &k2 = intrinsics[5];
        const auto &k3 = intrinsics[8];
        const auto &p1 = intrinsics[6];
        const auto &p2 = intrinsics[7];

        // 1. Apply the board transformation in teh object
        T point[3] = {T(x), T(y), T(z)};
        if (refine_board) {
            T point_refine[3];
            ceres::AngleAxisRotatePoint(boardPose, point, point_refine);
            std::copy_n(point_refine, 3, point);
            point[0] += boardPose[3];
            point[1] += boardPose[4];
            point[2] += boardPose[5];
        }

        // 2. apply transformation to the object (to expressed in the current
        // camera)
        T pobj[3];
        ceres::AngleAxisRotatePoint(objectPose, point, pobj);
        pobj[0] += objectPose[3];
        pobj[1] += objectPose[4];
        pobj[2] += objectPose[5];

        // 3. Refine the camera if it is not the referential
        if (refine_camera) {
            T pobj_refine[3];
            ceres::AngleAxisRotatePoint(cameraPose, pobj, pobj_refine);
            std::copy_n(pobj_refine, 3, pobj);
            pobj[0] += cameraPose[3];
            pobj[1] += cameraPose[4];
            pobj[2] += cameraPose[5];
        }

        pobj[0] /= pobj[2];
        pobj[1] /= pobj[2];

        if (distortion_type == 0) // perspective brown
        {
            T r2 = pobj[0] * pobj[0] + pobj[1] * pobj[1];
            T r4 = r2 * r2;
            T r6 = r4 * r2;
            T r_coeff = (T(1) + k1 * r2 + k2 * r4 + k3 * r6);
            T xd = pobj[0] * r_coeff + T(2) * p1 * pobj[0] * pobj[1] +
                   p2 * (r2 + T(2) * pobj[0] * pobj[0]);
            T yd = pobj[1] * r_coeff + p1 * (r2 + T(2) * (pobj[1] * pobj[1])) +
                   T(2) * p2 * pobj[0] * pobj[1];

            T up = T(fx) * xd + T(cx);
            T vp = T(fy) * yd + T(cy);

            residuals[0] = up - T(u);
            residuals[1] = vp - T(v);
        }

        if (distortion_type == 1) // fisheye
        {
            T r2 = pobj[0] * pobj[0] + pobj[1] * pobj[1];
            T r = sqrt(r2);
            T theta = atan(r);
            T theta2 = theta * theta, theta3 = theta2 * theta,
              theta4 = theta2 * theta2, theta5 = theta4 * theta;
            T theta6 = theta3 * theta3, theta7 = theta6 * theta,
              theta8 = theta4 * theta4, theta9 = theta8 * theta;
            T theta_d =
                theta + k1 * theta3 + k2 * theta5 + p1 * theta7 + p2 * theta9;
            T inv_r = r > T(1e-8) ? T(1) / r : T(1);
            T cdist = r > T(1e-8) ? theta_d * inv_r : T(1);

            T xd = pobj[0] * cdist;
            T yd = pobj[1] * cdist;

            T up = T(fx) * xd + T(cx);
            T vp = T(fy) * yd + T(cy);

            residuals[0] = up - T(u);
            residuals[1] = vp - T(v);
        }

        return true;
    }

    static ceres::CostFunction *create(double u, double v, double x, double y,
                                       double z, bool refine_camera,
                                       bool refine_board, int type)
    {
        return new ceres::AutoDiffCostFunction<
            ReprojectionError_CameraGroupAndObjectRefAndIntrinsics, 2, 6, 6, 6,
            9>(new ReprojectionError_CameraGroupAndObjectRefAndIntrinsics(
            u, v, x, y, z, refine_camera, refine_board, type));
    }
};

} // namespace tl::mcmb
