#pragma once

#include <array>

#include "esf_sampler.h"

class EsfQuadSampler : public EsfSampler
{
public:
    explicit EsfQuadSampler(double max_dot,
                            Bayer::cfa_mask_t cfa_mask = Bayer::ALL,
                            double border_width = 0.);

    void sample(Edge_model& edge_model,
                std::vector<Ordered_point>& local_ordered,
                const std::map<int, scanline>& scanset, double& edge_length,
                const cv::Mat& geom_img, const cv::Mat& sampling_img,
                Bayer::cfa_mask_t cfa_mask = Bayer::DEFAULT) override;

protected:
    void quad_tangency(const cv::Point2d& p, const std::array<double, 3>& qp,
                       std::vector<double>& roots);
};
