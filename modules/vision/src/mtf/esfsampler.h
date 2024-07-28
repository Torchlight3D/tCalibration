#pragma once

#include <map>

#include <opencv2/core/mat.hpp>

#include "orderedpoint.h"
#include "bayer.h"
#include "edgemodel.h"
#include "scanline.h"

class EsfSampler
{
public:
    enum esf_sampler_t
    {
        LINE = 0,
        QUAD,
        PIECEWISE_QUAD,
        DEFERRED
    };

    // TODO: Use magic_enum
    static esf_sampler_t from_string(const std::string &s);
    const static std::array<std::string, 4> esf_sampler_names;

    EsfSampler(double max_dot, Bayer::cfa_mask_t cfa_mask = Bayer::ALL,
               double max_edge_length = 1e6, double border_width = 0.);

    virtual void sample(Edge_model &edge_model,
                        std::vector<Ordered_point> &local_ordered,
                        const std::map<int, scanline> &scanset,
                        double &edge_length, const cv::Mat &geom_img,
                        const cv::Mat &sampling_img,
                        Bayer::cfa_mask_t cfa_mask = Bayer::DEFAULT) = 0;

protected:
    double max_dot;
    Bayer::cfa_mask_t default_cfa_mask;
    double max_edge_length;
    double border_width;
};
