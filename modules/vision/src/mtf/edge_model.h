#pragma once

#include <array>

#include <opencv2/core/types.hpp>

// Concept: an edge is modelled as a line (point on line and unit direction
// vector) The edge can be refined by populating the coefficients of a quadratic
// polynomial that allows for some curvature in the edge The ultimate definition
// of the edge is a dense set of points fitted locally to the gradient of the
// input image, called the "ridge"

// We can usually obtain estimates of the centroid, direction and quadratic
// coefficients during the quadrangle detection phase This information is not
// available in --single-roi mode, but we could extend that mode to produce a
// rough estimate

class Edge_model
{
public:
    Edge_model();
    Edge_model(const cv::Point2d& centroid, const cv::Point2d& direction);
    Edge_model(const cv::Point2d& centroid, const cv::Point2d& direction,
               const double coeffs[3]);

    void hint_point_set_size(int par_dist_bias, int max_edge_len,
                             int est_roi_width);

    void add_point(double x, double y, double weight,
                   double distance_threshold = 100);
    void estimate_ridge();

    void update_location(const cv::Point2d& new_centroid,
                         const cv::Point2d& new_direction);
    std::array<double, 3> quad_coeffs();

    cv::Point3d line_deviation();

    const cv::Point2d& get_centroid() const;

    const cv::Point2d& get_direction() const;

    const cv::Point2d& get_normal() const;

    bool quad_fit_valid() const;

public:
    std::vector<cv::Point2d> ridge;

private:
    void release_points();

    double pnorm(const cv::Point2d& dir) const;

    void fit_quad_to_ridge();

private:
    cv::Point2d centroid = {0, 0};
    cv::Point2d direction = {0, 0};
    cv::Point2d normal = {0, 0};
    std::array<double, 3> coeff; // least squares quadratic fit
    bool coeffs_invalidated = false;

    int par_bias = 0;
    bool points_hinted = false;
    std::vector<std::vector<cv::Point3d>> points;
};
