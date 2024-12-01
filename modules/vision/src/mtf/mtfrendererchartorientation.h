#pragma once

#include "mtfrenderer.h"
#include "distancescale.h"
#include "cameradraw.h"

class Mtf_renderer_chart_orientation : public Mtf_renderer
{
public:
    Mtf_renderer_chart_orientation(const std::string& img_filename,
                                   const std::string& wdir,
                                   const std::string& co_fname,
                                   const cv::Mat& img, int gnuplot_width,
                                   Distance_scale& distance_scale,
                                   cv::Rect* dimension_correction = nullptr);

    void render(const std::vector<Block>& blocks) override;

private:
    std::string wdir;
    std::string co_fname;

    cv::Rect* dimension_correction;

    Camera_draw draw;
};
