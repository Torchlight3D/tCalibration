#pragma once

#include "mtfrenderer.h"

class Mtf_renderer_profile : public Mtf_renderer
{
public:
    Mtf_renderer_profile(const std::string &img_filename,
                         const std::string &wdir, const std::string &prof_fname,
                         const std::string &gnuplot_binary, const cv::Mat &img,
                         int gnuplot_width, bool lpmm_mode = false,
                         double pixel_size = 1.0, int mtf_contrast = 50);

    void render(const std::vector<Block> &blocks) override;

    void set_gnuplot_warning(bool gnuplot);

    bool gnuplot_failed() const;

private:
    std::string wdir;
    std::string prname;
    std::string gnuplot_binary;
    const cv::Mat &img;
    bool lpmm_mode;
    double pixel_size;
    bool gnuplot_failure;
    bool gnuplot_warning;
    int gnuplot_width;
    std::string img_filename;
    int mtf_contrast = 50;
};
