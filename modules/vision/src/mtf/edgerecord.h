#pragma once

#include <vector>

#include <opencv2/core/types.hpp>

class Edge_record
{
public:
    Edge_record();

    static void pool_edges(Edge_record& a, Edge_record& b);

    bool compatible(const Edge_record& b);

    double relative_orientation(const Edge_record& b);

    void add_point(double x, double y, double gx, double gy);
    std::pair<double, double> compute_eigenvector_angle();

    bool reduce();
    bool is_pooled() const;

    void clear();

public:
    double slope;
    double angle;
    double rsq;
    cv::Point2d centroid;
    std::pair<double, double> radii;

private:
    void renormalize_weights();

private:
    std::vector<cv::Point2d> points;
    std::vector<double> weights;

    double wsum;

    double sB; // standard error in slope estimate

    bool pooled;
};
