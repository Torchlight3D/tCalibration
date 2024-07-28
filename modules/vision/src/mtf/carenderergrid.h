#pragma once

#include "mtfrenderer.h"

class Ca_renderer_grid : public Mtf_renderer
{
public:
    Ca_renderer_grid(const std::string& img_filename, const std::string& wdir,
                     const std::string& fname,
                     const std::string& gnuplot_binary, const cv::Mat& img,
                     int gnuplot_width, bool lpmm_mode, double pixel_size,
                     bool fraction_mode, bool allow_all_edges);

    void render(const std::vector<Block>& blocks) override;

    void set_sparse_chart(bool mode);

public:
    std::string img_filename;
    std::string wdir;
    std::string fname;
    std::string gnuplot_binary;
    cv::Mat img; // Maybe we can drop this one?
    int gnuplot_width;
    bool lpmm_mode;
    double pixel_size;
    bool fraction_mode;

    cv::Point2d img_centre;
    cv::Size img_dims;

    bool sparse_mode;
    bool allow_all_edges;

    std::vector<cv::Mat> grid_coarse;
    std::vector<cv::Mat> grid_fine;
};
