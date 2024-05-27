#pragma once

#include "ordered_point.h"
#include "mtf_renderer.h"

class Mtf_renderer_lensprofile : public Mtf_renderer
{
public:
    Mtf_renderer_lensprofile(
        const std::string& img_filename, const std::string& wdir,
        const std::string& prof_fname, const std::string& gnuplot_binary,
        const cv::Mat& img, const std::vector<double>& in_resolution,
        int gnuplot_width, bool lpmm_mode = false, double pixel_size = 1.0);

    void render(const std::vector<Block>& blocks) override;

    void set_sparse_chart(bool s);
    void set_fixed_size(bool f);

    void lsfit(const std::vector<Ordered_point>& in_data,
               std::vector<Ordered_point>& recon,
               std::vector<Ordered_point>& spread, int recon_samples = 64);

private:
    std::string wdir;
    std::string prname;
    std::string pfname;
    std::string gnuplot_binary;
    const cv::Mat& img;
    bool lpmm_mode;
    double pixel_size;
    bool gnuplot_failure;

    std::vector<double> in_resolution;
    int gnuplot_width;
    bool sparse_chart = false;
    bool fixed_size = false;
};
