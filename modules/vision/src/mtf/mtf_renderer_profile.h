#pragma once

#include "mtf_renderer.h"

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

    bool gnuplot_failed();

private:
    static double angular_diff(double a, double b);

    bool test_for_bimodal_distribution(const std::map<int, double> &data);

    double IQR(const std::vector<double> &counts, int start, int end);

    double median(const std::vector<double> &counts, int start, int end);

    void extract_row_maxima(std::map<int, double> &row_max,
                            const std::vector<Block> &blocks,
                            bool transpose = false);

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
