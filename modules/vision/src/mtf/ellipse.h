#pragma once

#include <set>

#include <Eigen/Dense>

#include "common_types.h"
#include "gradient.h"
#include "scanline.h"

inline constexpr double ellipse_fit_threshold = 0.2;
inline constexpr double ellipse_fit_deviation_threshold = 0.04;

inline constexpr double min_major_axis = 26 / 2.0;
// we expect a 50% reduction on a 45 degree slope
inline constexpr double min_minor_axis = min_major_axis / 3.0;
inline constexpr double max_ellipse_gradient_error = 15;

using iPoint = std::pair<int, int>;

class Component_labeller;

class Ellipse_detector
{
public:
    Ellipse_detector();
    ~Ellipse_detector();

    // TODO: Not implement?
    static cv::Point2d calculate_centroid(Pointlist& points);
    cv::Point2d centroid() const;

    int fit(const Component_labeller& cl, const Gradient& gradient,
            const Pointlist& raw_points, int tl_x, int tl_y, int dilate = 1);

    bool gradient_check(const Component_labeller& cl, const Gradient& gradient,
                        const Pointlist& raw_points);

    void set_code(int incode) { code = incode; }

    int _matrix_to_ellipse(Eigen::Matrix3d& C);
    void _dilate(std::set<iPoint>& s, int width, int height, int iters);
    void _dilate_outer_only(std::set<iPoint>& s, int width, int height);
    void _correct_eccentricity(double major_scale, double bp_x, double bp_y);

public:
    double centroid_x;
    double centroid_y;
    double major_axis;
    double minor_axis;
    double angle;
    double quality;
    double fg_fraction;
    bool valid; // flagged by detector at a later stage
    bool solid;
    int code;

    std::map<int, scanline> scanset;

    Eigen::Vector3d pos1;
    Eigen::Vector3d pos2;

    Eigen::Vector3d n1;
    Eigen::Vector3d n2;

private:
    Eigen::Matrix3d _C;
};
