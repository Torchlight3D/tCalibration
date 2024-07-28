#pragma once

#include <vector>

#include "bayer.h"
#include "cameradraw.h"
#include "distancescale.h"
#include "ellipse.h"
#include "mtfrenderer.h"
#include "mtfprofilesample.h"
#include "ratpolyfit.h"

class Mtf_renderer_focus : public Mtf_renderer
{
public:
    Mtf_renderer_focus(
        Distance_scale& distance_scale,
        const std::vector<std::pair<cv::Point2d, cv::Point2d>>& sliding_edges,
        const std::string& wdir, const std::string& prof_fname,
        const cv::Mat& img, [[maybe_unused]] bool lpmm_mode = false,
        [[maybe_unused]] double pixel_size = 1.0);

    void render(const std::vector<Block>&);

    void render(const std::vector<Mtf_profile_sample>& samples,
                Bayer::bayer_t bayer = Bayer::NONE,
                std::vector<Ellipse_detector>* ellipses = nullptr,
                cv::Rect* dimension_correction = nullptr);

private:
    Eigen::VectorXd rpfit(Ratpoly_fit& cf, bool scale = true,
                          bool refine = true);
    void exposure_checks(const cv::Point2d& dims, double& white_clip,
                         double& black_clip, double& overexposure);

private:
    cv::Point2d& zero;
    cv::Point2d& transverse;
    cv::Point2d& longitudinal;
    std::string wdir;
    std::string prname;
    const cv::Mat& img;
    Distance_scale& distance_scale;
    const std::vector<std::pair<cv::Point2d, cv::Point2d>>& sliding_edges;
    int initial_rows;

    double psf;

    Camera_draw draw;
};
