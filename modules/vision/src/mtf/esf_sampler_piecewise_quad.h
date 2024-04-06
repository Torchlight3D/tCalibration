#pragma once

#include "esf_sampler_quad.h"

class EsfPiecewiseQuadSampler : public EsfQuadSampler
{
public:
    explicit EsfPiecewiseQuadSampler(double max_dot,
                                     Bayer::cfa_mask_t cfa_mask = Bayer::ALL,
                                     double border_width = 0.);

    void sample(Edge_model& edge_model,
                std::vector<Ordered_point>& local_ordered,
                const std::map<int, scanline>& scanset, double& edge_length,
                const cv::Mat& geom_img, const cv::Mat& sampling_img,
                Bayer::cfa_mask_t cfa_mask = Bayer::DEFAULT) override;

protected:
    std::vector<double> piecewise_quadfit(const std::vector<cv::Point2d>& pts);
};
