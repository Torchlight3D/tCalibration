#pragma once

#include "edgerecord.h"
#include "gradient.h"

inline constexpr double rect_il_thresh = 7; // pixels, during initial linear fit
inline constexpr double quad_il_thresh = 2; // pixels
inline constexpr double quad_severe_outlier_thresh = 4; // pixels
const double quad_slope_thresh = tan(5.0 / 180.0 * M_PI);
const double quad_severe_slope_thresh = tan(15.0 / 180.0 * M_PI);
inline constexpr double quad_outlier_ratio_thresh = 0.1;
inline constexpr double quad_severe_outlier_ratio_thresh = 0.02;
inline constexpr double minimum_object_width = 14; // pixels
inline constexpr double adjust = 0.15;

class Intersection_record
{
public:
    Intersection_record(double distance, const cv::Point2d& intersection,
                        size_t index1, size_t index2)
        : distance(distance),
          intersection(intersection),
          index1(index1),
          index2(index2)
    {
    }

    bool operator<(const Intersection_record& b) const
    {
        return distance < b.distance;
    }

    double distance;
    cv::Point2d intersection;
    size_t index1;
    size_t index2;
};

class Mrectangle
{
public:
    Mrectangle();

    // build a rectangular buffer of "width" around midpoint of side k
    Mrectangle(const Mrectangle& b, size_t k, double width);

    // reposition a rectangle using new estimates of the centroids and normals
    Mrectangle(const Mrectangle& b,
               const std::vector<Edge_record>& edge_records);

    Mrectangle(const std::vector<double>& in_thetas,
               const std::vector<double>& data_thetas,
               const std::vector<cv::Point2d>& points, const Gradient& g,
               int label, double thresh = 5.0 / 180.0 * M_PI,
               bool allow_partial = false);

    bool corners_ok() const;

    bool is_inside(const cv::Point2d& p) const;

    cv::Point2d get_centroid(size_t i) const;

    void print() const;

public:
    std::vector<double> thetas;
    std::vector<cv::Point2d> centroids;
    bool valid;
    std::vector<cv::Point2d> corners;
    std::vector<cv::Point2d> edges;
    std::vector<cv::Point2d> normals;
    std::vector<std::vector<int>> corner_map;
    double quad_coeffs[4][3];
    cv::Point2d tl;
    cv::Point2d br;
    size_t boundary_length;
    double length = 0; // measured along major axis
    double area = 0;
    std::vector<cv::Point3d> line_deviation;
};
