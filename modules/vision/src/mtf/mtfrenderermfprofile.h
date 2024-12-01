#pragma once

#include <vector>

#include "distancescale.h"
#include "mtfprofilesample.h"
#include "mtfrenderer.h"

class Mtf_renderer_mfprofile : public Mtf_renderer
{
public:
    Mtf_renderer_mfprofile(Distance_scale& distance_scale,
                           const std::string& wdir,
                           const std::string& prof_fname, const cv::Mat& img,
                           bool lpmm_mode = false, double pixel_size = 1.0);

    void render(const std::vector<Block>&) override;

    void render(const std::vector<Mtf_profile_sample>& samples);

private:
    void draw_curve(cv::Mat& image, const std::vector<cv::Point2d>& data,
                    cv::Scalar col, double width, bool points = false);

private:
    std::string wdir;
    std::string prname;
    const cv::Mat& img;
    Distance_scale& distance_scale;
};
