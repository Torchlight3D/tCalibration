#pragma once

#include <vector>

#include "distance_scale.h"
#include "mtf_profile_sample.h"
#include "ratpoly_fit.h"

class Focus_surface
{
public:
    Focus_surface(std::vector<Mtf_profile_sample> &data, int order_n,
                  int order_m, Distance_scale &distance_scale);

    Eigen::VectorXd rpfit(Ratpoly_fit &cf, bool scale = true,
                          bool refine = true);

    double evaluate(Eigen::VectorXd &);

    int dimension();

    double softclamp(double x, double lower, double upper, double p = 0.98);

public:
    std::vector<Mtf_profile_sample> &data;
    int order_n;
    int order_m;
    double maxy;
    double maxx;
    std::vector<cv::Point2d> ridge;
    std::vector<cv::Point2d> ridge_peaks;
    std::vector<cv::Point2d> ridge_p05;
    std::vector<cv::Point2d> ridge_p95;
    double focus_peak;
    double focus_peak_p05;
    double focus_peak_p95;

    double p2;
    double p98;
};
