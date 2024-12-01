#pragma once

#include "esfsampler.h"
#include "undistort.h"

class EsfDeferredSampler : public EsfSampler
{
public:
    EsfDeferredSampler(Undistort* undistort, double max_dot,
                       Bayer::cfa_mask_t cfa_mask = Bayer::ALL,
                       double border_width = 0.);

    void sample(Edge_model& edge_model,
                std::vector<Ordered_point>& local_ordered,
                const std::map<int, scanline>& scanset, double& edge_length,
                const cv::Mat& geom_img, const cv::Mat& sampling_img,
                Bayer::cfa_mask_t cfa_mask = Bayer::DEFAULT) override;

protected:
    cv::Point2d bracket_minimum(double t0, const cv::Point2d& l,
                                const cv::Point2d& p, const cv::Point2d& pt);
    cv::Point2d derivative(double t0, const cv::Point2d& l,
                           const cv::Point2d& p);

private:
    Undistort* undistort_{nullptr};
};
