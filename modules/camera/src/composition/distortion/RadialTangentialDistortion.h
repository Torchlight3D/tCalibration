﻿#pragma once

#include "DistortionInterface.h"
#include "../core/eigen_types.h"

namespace tl {

class RadialTangentialDistortion
{
public:
    enum
    {
        IntrinsicsDimension = 4
    };
    enum
    {
        DesignVariableDimension = IntrinsicsDimension
    };

    RadialTangentialDistortion();
    RadialTangentialDistortion(double k1, double k2, double p1, double p2);
    virtual ~RadialTangentialDistortion();

    // parameters
    double k1() const { return _k1; }
    double k2() const { return _k2; }
    double p1() const { return _p1; }
    double p2() const { return _p2; }

    void setParameters(const Eigen::MatrixXd& params);
    void parameters(Eigen::MatrixXd& params) const;
    Eigen::Vector2i parameterSize() const;

    void clear();

    // point mapping
    template <typename Derived_p>
    void distort(const Eigen::MatrixBase<Derived_p>& normalizedPoint) const;

    template <typename Derived_p, typename Derived_j>
    void distort(const Eigen::MatrixBase<Derived_p>& normalizedPoint,
                 const Eigen::MatrixBase<Derived_j>& distortionJacobian) const;

    template <typename Derived_p>
    void undistort(const Eigen::MatrixBase<Derived_p>& normalizedPoint) const;

    template <typename Derived_p, typename Derived_j>
    void undistort(
        const Eigen::MatrixBase<Derived_p>& y,
        const Eigen::MatrixBase<Derived_j>& undistortionJacobian) const;

    template <typename Derived_p, typename Derived_j>
    void distortParameterJacobian(
        const Eigen::MatrixBase<Derived_p>& point,
        const Eigen::MatrixBase<Derived_j>& distortionJacobian) const;

    // slam backend compatibility
    void update(const double* v);

    int minimalDimensions() const;

private:
    double _k1{0.}, _k2{0.}, _p1{0.}, _p2{0.};
};

template <typename DERIVED_Y>
void RadialTangentialDistortion::distort(
    const Eigen::MatrixBase<DERIVED_Y>& yconst) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<DERIVED_Y>, 2);

    auto y = const_cast<Eigen::MatrixBase<DERIVED_Y>&>(yconst);
    y.derived().resize(2);

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = y[0] * y[0];
    my2_u = y[1] * y[1];
    mxy_u = y[0] * y[1];
    rho2_u = mx2_u + my2_u;
    rad_dist_u = _k1 * rho2_u + _k2 * rho2_u * rho2_u;
    y[0] +=
        y[0] * rad_dist_u + 2.0 * _p1 * mxy_u + _p2 * (rho2_u + 2.0 * mx2_u);
    y[1] +=
        y[1] * rad_dist_u + 2.0 * _p2 * mxy_u + _p1 * (rho2_u + 2.0 * my2_u);
}

template <typename DERIVED_Y, typename DERIVED_JY>
void RadialTangentialDistortion::distort(
    const Eigen::MatrixBase<DERIVED_Y>& yconst,
    const Eigen::MatrixBase<DERIVED_JY>& outJy) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<DERIVED_Y>, 2);
    EIGEN_STATIC_ASSERT_MATRIX_SIZE(Eigen::MatrixBase<DERIVED_JY>, 2, 2);

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    auto J = const_cast<Eigen::MatrixBase<DERIVED_JY>&>(outJy);
    J.derived().resize(2, 2);
    J.setZero();

    auto y = const_cast<Eigen::MatrixBase<DERIVED_Y>&>(yconst);
    y.derived().resize(2);

    mx2_u = y[0] * y[0];
    my2_u = y[1] * y[1];
    mxy_u = y[0] * y[1];
    rho2_u = mx2_u + my2_u;

    rad_dist_u = _k1 * rho2_u + _k2 * rho2_u * rho2_u;

    J(0, 0) = 1 + rad_dist_u + _k1 * 2.0 * mx2_u + _k2 * rho2_u * 4 * mx2_u +
              2.0 * _p1 * y[1] + 6 * _p2 * y[0];
    J(1, 0) = _k1 * 2.0 * y[0] * y[1] + _k2 * 4 * rho2_u * y[0] * y[1] +
              _p1 * 2.0 * y[0] + 2.0 * _p2 * y[1];
    J(0, 1) = J(1, 0);
    J(1, 1) = 1 + rad_dist_u + _k1 * 2.0 * my2_u + _k2 * rho2_u * 4 * my2_u +
              6 * _p1 * y[1] + 2.0 * _p2 * y[0];

    y[0] +=
        y[0] * rad_dist_u + 2.0 * _p1 * mxy_u + _p2 * (rho2_u + 2.0 * mx2_u);
    y[1] +=
        y[1] * rad_dist_u + 2.0 * _p2 * mxy_u + _p1 * (rho2_u + 2.0 * my2_u);
}

template <typename DERIVED>
void RadialTangentialDistortion::undistort(
    const Eigen::MatrixBase<DERIVED>& yconst) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<DERIVED>, 2);

    auto y = const_cast<Eigen::MatrixBase<DERIVED>&>(yconst);
    y.derived().resize(2);

    Eigen::Vector2d ybar = y;
    const int n = 5;
    Eigen::Matrix2d F;

    Eigen::Vector2d y_tmp;
    for (int i = 0; i < n; i++) {
        y_tmp = ybar;

        distort(y_tmp, F);

        Eigen::Vector2d e(y - y_tmp);
        Eigen::Vector2d du = (F.transpose() * F).inverse() * F.transpose() * e;

        ybar += du;

        if (e.dot(e) < 1e-15)
            break;
    }
    y = ybar;
}

template <typename DERIVED, typename DERIVED_JY>
void RadialTangentialDistortion::undistort(
    const Eigen::MatrixBase<DERIVED>& yconst,
    const Eigen::MatrixBase<DERIVED_JY>& outJy) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<DERIVED>, 2);
    EIGEN_STATIC_ASSERT_MATRIX_SIZE(Eigen::MatrixBase<DERIVED_JY>, 2, 2);

    auto y = const_cast<Eigen::MatrixBase<DERIVED>&>(yconst);
    y.derived().resize(2);

    // we use f^-1 ' = ( f'(f^-1) ) '
    // with f^-1 the undistortion
    // and  f the distortion
    undistort(y); // first get the undistorted image

    Eigen::Vector2d kp = y;
    Eigen::Matrix2d Jd;
    distort(kp, Jd);

    // now y = f^-1(y0)
    DERIVED_JY& J = const_cast<DERIVED_JY&>(outJy.derived());

    J = Jd.inverse();

    /*  std::cout << "J: " << std::endl << J << std::endl;

    double mx2_u = y[0]*y[0];
    double my2_u = y[1]*y[1];
    //double mxy_u = y[0]*y[1];
    double rho2_u = mx2_u+my2_u;

     double rad_dist_u = _k1*rho2_u+_k2*rho2_u*rho2_u;
     // take the inverse as Jacobian.
     J(0,0) = 1/(1 + rad_dist_u + _k1*2*mx2_u + _k2*rho2_u*4*mx2_u + 2*_p1*y[1]
    + 6*_p2*y[0]); J(1,0) = (_k1*2*y[0]*y[1] + _k2*4*rho2_u*y[0]*y[1] +
    _p1*2*y[0] + 2*_p2*y[1]); J(0,1) = J(1,0); J(1,1) = 1/(1 + rad_dist_u +
    _k1*2*my2_u + _k2*rho2_u*4*my2_u + 6*_p1*y[1] + 2*_p2*y[0]);

    // this should only happen if the distortion coefficients are 0
    // the coefficients being zero removes the cross dependence then it is safe
    to set J(1,0) = 0 if (J(1,0) != 0) J(1,0) = 1/J(1,0); J(0,1) = J(1,0);*/
}

template <typename DERIVED_Y, typename DERIVED_JD>
void RadialTangentialDistortion::distortParameterJacobian(
    const Eigen::MatrixBase<DERIVED_Y>& imageY,
    const Eigen::MatrixBase<DERIVED_JD>& outJd) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<DERIVED_Y>, 2);
    EIGEN_STATIC_ASSERT_MATRIX_SIZE(Eigen::MatrixBase<DERIVED_JD>, 2, 4);

    double y0 = imageY[0];
    double y1 = imageY[1];
    double r2 = y0 * y0 + y1 * y1;
    double r4 = r2 * r2;

    auto J = const_cast<Eigen::MatrixBase<DERIVED_JD>&>(outJd);
    J.derived().resize(2, 4);
    J.setZero();

    J(0, 0) = y0 * r2;
    J(0, 1) = y0 * r4;
    J(0, 2) = 2.0 * y0 * y1;
    J(0, 3) = r2 + 2.0 * y0 * y0;

    J(1, 0) = y1 * r2;
    J(1, 1) = y1 * r4;
    J(1, 2) = r2 + 2.0 * y1 * y1;
    J(1, 3) = 2.0 * y0 * y1;
}

} // namespace tl
