#pragma once

#include "ellipsepoint.hpp"

namespace tl {
namespace runetag {

class EllipseFitter
{
public:
    cv::Matx33d VR;

private:
    const double coplanar_threshold;
    const double size_err_threshold;

    const EllipsePoint& e1;
    const EllipsePoint& e2;
    const cv::Mat& intrinsics;

    mutable cv::Matx33d Qfit1c;
    mutable cv::Matx33d Qfit2c;

    double rad_avg;
    double radius;

    /*
    Main operation provided by getFit1WithOffset and getFit2WithOffset
    */
    cv::Matx33d getFitWithOffset(double rad_mul, cv::Matx33d& Qfit,
                                 double k) const;

    bool chooseAvgVR(const EllipsePoint& ep1, const EllipsePoint& ep2,
                     double& r, double& r_other, cv::Matx33d& Qc,
                     cv::Matx33d& Qc_other);

    // AUX OPERATION
    static cv::Matx33d& transformToCircle(const cv::Matx33d& Q,
                                          const cv::Matx33d& VR,
                                          cv::Matx33d& targetMatrix);
    static void normalize(cv::Matx31d& vector);
    static cv::Matx31d buildN(const cv::Matx33d& VR);
    static double circleRonZ0(const cv::Matx33d& Qc);
    static bool fitCircles(const cv::Point2d& center1,
                           const cv::Point2d& center2, double radius2,
                           cv::Matx33d& circle1, cv::Matx33d& circle2);
    static void buildCircle(cv::Matx33d& target, double a, double b,
                            double radius);

    // AUX STRUCTURE
    class CandidateVR
    {
    private:
        const cv::Matx33d* VR1;
        const cv::Matx33d* VR2;
        const cv::Matx31d* normal1;
        const cv::Matx31d* normal2;

    public:
        CandidateVR(const cv::Matx33d& _VR1, const cv::Matx31d& _normal1,
                    const cv::Matx33d& _VR2, const cv::Matx31d& _normal2);

        inline const cv::Matx33d& getVR1() const { return *VR1; }

        inline const cv::Matx33d& getVR2() const { return *VR2; }

        inline const cv::Matx31d& getNormal1() const { return *normal1; }

        inline const cv::Matx31d& getNormal2() const { return *normal2; }

        double cosAngle() const;
    };

public:
    EllipseFitter(const EllipsePoint& _e1, const EllipsePoint& _e2,
                  const cv::Mat& _intrinsics, double _coplanar_threshold = 0.97,
                  double _size_err_threshold = 0.5)
        : e1(_e1),
          e2(_e2),
          intrinsics(_intrinsics),
          coplanar_threshold(_coplanar_threshold),
          size_err_threshold(_size_err_threshold)
    {
    }

    inline const cv::Matx33d& getVR() const { return VR; }

    // MAIN OPERATIONS
    bool fitEllipseAvg(double _radius_ratio);
    cv::Matx33d getFit1WithOffset(double rad_mul) const;
    cv::Matx33d getFit2WithOffset(double rad_mul) const;
};

} // namespace runetag
} // namespace tl
