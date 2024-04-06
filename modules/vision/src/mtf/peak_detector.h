#pragma once

#include <stdlib.h>
#include <vector>
#include <queue>

#define DBG(x)

class Peak_detector
{
public:
    Peak_detector(const std::vector<double>& in_data, size_t in_bins,
                  double in_min_data = -M_PI, double in_max_data = M_PI);

    void select_best_n(std::vector<double>& best, size_t in_n,
                       double il_thresh = 0.03);

    // TODO: Duplicated code
    static double angular_diff(double a, double b);

private:
    size_t nbins;
    const std::vector<double>& data;
    std::vector<size_t> counts;
    double min_data;
    double max_data;
    double range;

    std::priority_queue<std::pair<size_t, double>> pq;
};
