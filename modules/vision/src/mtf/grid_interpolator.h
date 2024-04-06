#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "block.h"

enum Edge_type
{
    MERIDIONAL,
    SAGITTAL,
    NEITHER
};

class Grid_functor
{
public:
    virtual double value(const Block& /*block*/, size_t /*edge*/) const
    {
        return 0;
    }

    virtual bool in_range(double /*val*/) const { return true; }

    virtual double clamp(double value, double /*upper*/, double /*lower*/) const
    {
        return value;
    }

    virtual double nodata() const { return 0; }

    virtual double smoothing_factor() const { return 1e-3; }

    virtual int pruning_threshold() const { return 1; }
};

void interpolate_grid(const Grid_functor& ftor, Edge_type target_edge_type,
                      cv::Mat& grid_coarse, cv::Mat& grid_fine,
                      cv::Size img_dims, const std::vector<Block>& blocks,
                      double upper, double smoothing_factor = 1e-3,
                      int pruning_threshold = 1);
