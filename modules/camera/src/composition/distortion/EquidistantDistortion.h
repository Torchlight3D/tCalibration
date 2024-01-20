#pragma once

#include "../../../core/eigen_types.h"

namespace tl {

class EquidistantDistortion
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

    EquidistantDistortion();
    EquidistantDistortion(double k1, double k2, double k3, double k4);
    virtual ~EquidistantDistortion();

    // parameters
    double k1() const { return _k1; }
    double k2() const { return _k2; }
    double k3() const { return _k3; }
    double k4() const { return _k4; }

    void setParameters(const Eigen::MatrixXd& params);
    void parameters(Eigen::MatrixXd& params) const;
    Eigen::Vector2i parameterSize() const;

    void clear();

    // point mapping
    template <typename Derived_p>
    void distort(Eigen::MatrixBase<Derived_p>& normalizedPoint) const;

    template <typename Derived_p, typename Derived_j>
    void distort(Eigen::MatrixBase<Derived_p>& normalizedPoint,
                 Eigen::MatrixBase<Derived_j>& distortionJacobian) const;

    template <typename Derived_p>
    void undistort(Eigen::MatrixBase<Derived_p>& normalizedPoint) const;

    template <typename Derived_p, typename Derived_j>
    void undistort(Eigen::MatrixBase<Derived_p>& y,
                   Eigen::MatrixBase<Derived_j>& undistortionJacobian) const;

    template <typename Derived_p, typename Derived_j>
    void distortParameterJacobian(
        const Eigen::MatrixBase<Derived_p>& point,
        const Eigen::MatrixBase<Derived_j>& distortionJacobian) const;

    // slam backend compatibility
    // QUESTION: what is v?
    void update(const double* v);

    int minimalDimensions() const;

private:
    double _k1{0.}, _k2{0.}, _k3{0.}, _k4{0.};
};

///------- Implementation

template <typename Derived_p>
void EquidistantDistortion::distort(Eigen::MatrixBase<Derived_p>& pt) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<Derived_p>, 2);

    pt.derived().resize(2);

    const double r = sqrt(pt[0] * pt[0] + pt[1] * pt[1]);
    const double theta = atan(r);
    const double theta2 = theta * theta;
    const double theta4 = theta2 * theta2;
    const double theta6 = theta4 * theta2;
    const double theta8 = theta4 * theta4;
    const double thetad =
        theta * (1 + _k1 * theta2 + _k2 * theta4 + _k3 * theta6 + _k4 * theta8);

    const double scaling = (r > 1e-8) ? thetad / r : 1.0;

    pt[0] *= scaling;
    pt[1] *= scaling;
}

template <typename Derived_p, typename Derived_j>
void EquidistantDistortion::distort(Eigen::MatrixBase<Derived_p>& pt,
                                    Eigen::MatrixBase<Derived_j>& J) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<Derived_p>, 2);
    EIGEN_STATIC_ASSERT_MATRIX_SIZE(Eigen::MatrixBase<Derived_j>, 2, 2);

    // double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;    // ?

    J.derived().resize(2, 2);
    J.setZero();

    // MATLAB generated Jacobian
    J(0, 0) =
        atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])) * 1.0 /
            sqrt(pt[0] * pt[0] + pt[1] * pt[1]) *
            (_k1 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 2.0) +
             _k2 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 4.0) +
             _k3 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 6.0) +
             _k4 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 8.0) + 1.0) +
        pt[0] * atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])) * 1.0 /
            sqrt(pt[0] * pt[0] + pt[1] * pt[1]) *
            ((_k2 * pt[0] *
              pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 3.0) * 1.0 /
              sqrt(pt[0] * pt[0] + pt[1] * pt[1]) * 4.0) /
                 (pt[0] * pt[0] + pt[1] * pt[1] + 1.0) +
             (_k3 * pt[0] *
              pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 5.0) * 1.0 /
              sqrt(pt[0] * pt[0] + pt[1] * pt[1]) * 6.0) /
                 (pt[0] * pt[0] + pt[1] * pt[1] + 1.0) +
             (_k4 * pt[0] *
              pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 7.0) * 1.0 /
              sqrt(pt[0] * pt[0] + pt[1] * pt[1]) * 8.0) /
                 (pt[0] * pt[0] + pt[1] * pt[1] + 1.0) +
             (_k1 * pt[0] * atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])) * 1.0 /
              sqrt(pt[0] * pt[0] + pt[1] * pt[1]) * 2.0) /
                 (pt[0] * pt[0] + pt[1] * pt[1] + 1.0)) +
        ((pt[0] * pt[0]) *
         (_k1 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 2.0) +
          _k2 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 4.0) +
          _k3 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 6.0) +
          _k4 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 8.0) + 1.0)) /
            ((pt[0] * pt[0] + pt[1] * pt[1]) *
             (pt[0] * pt[0] + pt[1] * pt[1] + 1.0)) -
        (pt[0] * pt[0]) * atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])) * 1.0 /
            pow(pt[0] * pt[0] + pt[1] * pt[1], 3.0 / 2.0) *
            (_k1 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 2.0) +
             _k2 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 4.0) +
             _k3 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 6.0) +
             _k4 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 8.0) + 1.0);
    J(0, 1) =
        pt[0] * atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])) * 1.0 /
            sqrt(pt[0] * pt[0] + pt[1] * pt[1]) *
            ((_k2 * pt[1] *
              pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 3.0) * 1.0 /
              sqrt(pt[0] * pt[0] + pt[1] * pt[1]) * 4.0) /
                 (pt[0] * pt[0] + pt[1] * pt[1] + 1.0) +
             (_k3 * pt[1] *
              pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 5.0) * 1.0 /
              sqrt(pt[0] * pt[0] + pt[1] * pt[1]) * 6.0) /
                 (pt[0] * pt[0] + pt[1] * pt[1] + 1.0) +
             (_k4 * pt[1] *
              pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 7.0) * 1.0 /
              sqrt(pt[0] * pt[0] + pt[1] * pt[1]) * 8.0) /
                 (pt[0] * pt[0] + pt[1] * pt[1] + 1.0) +
             (_k1 * pt[1] * atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])) * 1.0 /
              sqrt(pt[0] * pt[0] + pt[1] * pt[1]) * 2.0) /
                 (pt[0] * pt[0] + pt[1] * pt[1] + 1.0)) +
        (pt[0] * pt[1] *
         (_k1 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 2.0) +
          _k2 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 4.0) +
          _k3 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 6.0) +
          _k4 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 8.0) + 1.0)) /
            ((pt[0] * pt[0] + pt[1] * pt[1]) *
             (pt[0] * pt[0] + pt[1] * pt[1] + 1.0)) -
        pt[0] * pt[1] * atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])) * 1.0 /
            pow(pt[0] * pt[0] + pt[1] * pt[1], 3.0 / 2.0) *
            (_k1 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 2.0) +
             _k2 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 4.0) +
             _k3 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 6.0) +
             _k4 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 8.0) + 1.0);
    J(1, 0) =
        pt[1] * atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])) * 1.0 /
            sqrt(pt[0] * pt[0] + pt[1] * pt[1]) *
            ((_k2 * pt[0] *
              pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 3.0) * 1.0 /
              sqrt(pt[0] * pt[0] + pt[1] * pt[1]) * 4.0) /
                 (pt[0] * pt[0] + pt[1] * pt[1] + 1.0) +
             (_k3 * pt[0] *
              pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 5.0) * 1.0 /
              sqrt(pt[0] * pt[0] + pt[1] * pt[1]) * 6.0) /
                 (pt[0] * pt[0] + pt[1] * pt[1] + 1.0) +
             (_k4 * pt[0] *
              pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 7.0) * 1.0 /
              sqrt(pt[0] * pt[0] + pt[1] * pt[1]) * 8.0) /
                 (pt[0] * pt[0] + pt[1] * pt[1] + 1.0) +
             (_k1 * pt[0] * atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])) * 1.0 /
              sqrt(pt[0] * pt[0] + pt[1] * pt[1]) * 2.0) /
                 (pt[0] * pt[0] + pt[1] * pt[1] + 1.0)) +
        (pt[0] * pt[1] *
         (_k1 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 2.0) +
          _k2 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 4.0) +
          _k3 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 6.0) +
          _k4 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 8.0) + 1.0)) /
            ((pt[0] * pt[0] + pt[1] * pt[1]) *
             (pt[0] * pt[0] + pt[1] * pt[1] + 1.0)) -
        pt[0] * pt[1] * atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])) * 1.0 /
            pow(pt[0] * pt[0] + pt[1] * pt[1], 3.0 / 2.0) *
            (_k1 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 2.0) +
             _k2 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 4.0) +
             _k3 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 6.0) +
             _k4 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 8.0) + 1.0);
    J(1, 1) =
        atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])) * 1.0 /
            sqrt(pt[0] * pt[0] + pt[1] * pt[1]) *
            (_k1 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 2.0) +
             _k2 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 4.0) +
             _k3 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 6.0) +
             _k4 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 8.0) + 1.0) +
        pt[1] * atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])) * 1.0 /
            sqrt(pt[0] * pt[0] + pt[1] * pt[1]) *
            ((_k2 * pt[1] *
              pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 3.0) * 1.0 /
              sqrt(pt[0] * pt[0] + pt[1] * pt[1]) * 4.0) /
                 (pt[0] * pt[0] + pt[1] * pt[1] + 1.0) +
             (_k3 * pt[1] *
              pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 5.0) * 1.0 /
              sqrt(pt[0] * pt[0] + pt[1] * pt[1]) * 6.0) /
                 (pt[0] * pt[0] + pt[1] * pt[1] + 1.0) +
             (_k4 * pt[1] *
              pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 7.0) * 1.0 /
              sqrt(pt[0] * pt[0] + pt[1] * pt[1]) * 8.0) /
                 (pt[0] * pt[0] + pt[1] * pt[1] + 1.0) +
             (_k1 * pt[1] * atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])) * 1.0 /
              sqrt(pt[0] * pt[0] + pt[1] * pt[1]) * 2.0) /
                 (pt[0] * pt[0] + pt[1] * pt[1] + 1.0)) +
        ((pt[1] * pt[1]) *
         (_k1 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 2.0) +
          _k2 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 4.0) +
          _k3 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 6.0) +
          _k4 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 8.0) + 1.0)) /
            ((pt[0] * pt[0] + pt[1] * pt[1]) *
             (pt[0] * pt[0] + pt[1] * pt[1] + 1.0)) -
        (pt[1] * pt[1]) * atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])) * 1.0 /
            pow(pt[0] * pt[0] + pt[1] * pt[1], 3.0 / 2.0) *
            (_k1 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 2.0) +
             _k2 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 4.0) +
             _k3 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 6.0) +
             _k4 * pow(atan(sqrt(pt[0] * pt[0] + pt[1] * pt[1])), 8.0) + 1.0);

    distort(pt);
}

template <typename Derived_p>
void EquidistantDistortion::undistort(Eigen::MatrixBase<Derived_p>& pt) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<Derived_p>, 2);

    pt.derived().resize(2);

    const double thetad = sqrt(pt[0] * pt[0] + pt[1] * pt[1]);

    double theta{thetad};
    double theta2, theta4, theta6, theta8;
    for (int i = 20; i > 0; i--) {
        theta2 = theta * theta;
        theta4 = theta2 * theta2;
        theta6 = theta4 * theta2;
        theta8 = theta4 * theta4;
        theta = thetad /
                (1 + _k1 * theta2 + _k2 * theta4 + _k3 * theta6 + _k4 * theta8);
    }
    const double scaling = tan(theta) / thetad;

    pt[0] *= scaling;
    pt[1] *= scaling;
}

template <typename Derived_p, typename Derived_j>
void EquidistantDistortion::undistort(Eigen::MatrixBase<Derived_p>& point,
                                      Eigen::MatrixBase<Derived_j>& J) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<Derived_p>, 2);
    EIGEN_STATIC_ASSERT_MATRIX_SIZE(Eigen::MatrixBase<Derived_j>, 2, 2);

    auto y = const_cast<Eigen::MatrixBase<Derived_p>&>(point);
    y.derived().resize(2);

    // we use f^-1 ' = ( f'(f^-1) ) '
    // with f^-1 the undistortion
    // and  f the distortion
    undistort(y); // first get the undistorted image

    Eigen::Vector2d kp = y;
    Eigen::Matrix2d Jd;
    distort(kp, Jd);

    // now y = f^-1(y0)
    J = Jd.inverse();
}

template <typename Derived_p, typename Derived_j>
void EquidistantDistortion::distortParameterJacobian(
    const Eigen::MatrixBase<Derived_p>& point,
    const Eigen::MatrixBase<Derived_j>& Jacobian) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<Derived_p>, 2);
    EIGEN_STATIC_ASSERT_MATRIX_SIZE(Eigen::MatrixBase<Derived_j>, 2, 4);

    auto J = const_cast<Eigen::MatrixBase<Derived_j>&>(Jacobian);
    J.derived().resize(2, 4);
    J.setZero();

    J(0, 0) = point[0] *
              pow(atan(sqrt(point[0] * point[0] + point[1] * point[1])), 3.0) *
              1.0 / sqrt(point[0] * point[0] + point[1] * point[1]);
    J(0, 1) = point[0] *
              pow(atan(sqrt(point[0] * point[0] + point[1] * point[1])), 5.0) *
              1.0 / sqrt(point[0] * point[0] + point[1] * point[1]);
    J(0, 2) = point[0] *
              pow(atan(sqrt(point[0] * point[0] + point[1] * point[1])), 7.0) *
              1.0 / sqrt(point[0] * point[0] + point[1] * point[1]);
    J(0, 3) = point[0] *
              pow(atan(sqrt(point[0] * point[0] + point[1] * point[1])), 9.0) *
              1.0 / sqrt(point[0] * point[0] + point[1] * point[1]);
    J(1, 0) = point[1] *
              pow(atan(sqrt(point[0] * point[0] + point[1] * point[1])), 3.0) *
              1.0 / sqrt(point[0] * point[0] + point[1] * point[1]);
    J(1, 1) = point[1] *
              pow(atan(sqrt(point[0] * point[0] + point[1] * point[1])), 5.0) *
              1.0 / sqrt(point[0] * point[0] + point[1] * point[1]);
    J(1, 2) = point[1] *
              pow(atan(sqrt(point[0] * point[0] + point[1] * point[1])), 7.0) *
              1.0 / sqrt(point[0] * point[0] + point[1] * point[1]);
    J(1, 3) = point[1] *
              pow(atan(sqrt(point[0] * point[0] + point[1] * point[1])), 9.0) *
              1.0 / sqrt(point[0] * point[0] + point[1] * point[1]);

    /*
     double y0 = imageY[0];
     double y1 = imageY[1];
     double r2 = y0*y0 + y1*y1;
     double r4 = r2*r2;

    Eigen::MatrixBase<Derived_j> & J = const_cast<Eigen::MatrixBase<Derived_j>
    &>(outJd); J.derived().resize(2,4); J.setZero();

     J(0,0) = y0*r2;
     J(0,1) = y0*r4;
     J(0,2) = 2.0*y0*y1;
     J(0,3) = r2 + 2.0*y0*y0;

    J(1,0) = y1*r2;
    J(1,1) = y1*r4;
    J(1,2) = r2 + 2.0*y1*y1;
    J(1,3) = 2.0*y0*y1;
    */
}

} // namespace tl
