#pragma once

#include <Eigen/Core>

#include "mtfrenderer.h"

// Moreland's diverging colour map:
// Moreland, K., Diverging Color Maps for Scientific Visualization, In
// Proceedings of the 5th International Symposium on Visual Computing, December
// 2009.

const std::string diverging_palette = "set palette defined(\
0       0.2314  0.2980  0.7529,\
0.03125 0.2667  0.3529  0.8000,\
0.0625  0.3020  0.4078  0.8431,\
0.09375 0.3412  0.4588  0.8824,\
0.125   0.3843  0.5098  0.9176,\
0.15625 0.4235  0.5569  0.9451,\
0.1875  0.4667  0.6039  0.9686,\
0.21875 0.5098  0.6471  0.9843,\
0.25    0.5529  0.6902  0.9961,\
0.28125 0.5961  0.7255  1.0000,\
0.3125  0.6392  0.7608  1.0000,\
0.34375 0.6824  0.7882  0.9922,\
0.375   0.7216  0.8157  0.9765,\
0.40625 0.7608  0.8353  0.9569,\
0.4375  0.8000  0.8510  0.9333,\
0.46875 0.8353  0.8588  0.9020,\
0.5     0.8667  0.8667  0.8667,\
0.53125 0.8980  0.8471  0.8196,\
0.5625  0.9255  0.8275  0.7725,\
0.59375 0.9451  0.8000  0.7255,\
0.625   0.9608  0.7686  0.6784,\
0.65625 0.9686  0.7333  0.6275,\
0.6875  0.9686  0.6941  0.5804,\
0.71875 0.9686  0.6510  0.5294,\
0.75    0.9569  0.6039  0.4824,\
0.78125 0.9451  0.5529  0.4353,\
0.8125  0.9255  0.4980  0.3882,\
0.84375 0.8980  0.4392  0.3451,\
0.875   0.8706  0.3765  0.3020,\
0.90625 0.8353  0.3137  0.2588,\
0.9375  0.7961  0.2431  0.2196,\
0.96875 0.7529  0.1569  0.1843,\
1       0.7059  0.0157  0.1490\
)";

class Mtf_renderer_grid : public Mtf_renderer
{
public:
    Mtf_renderer_grid(const std::string& img_filename, const std::string& wdir,
                      const std::string& fname,
                      const std::string& gnuplot_binary, const cv::Mat& img,
                      int gnuplot_width, bool lpmm_mode, double pixel_size,
                      double in_zscale, double surface_max = -1,
                      int mtf_contrast = 50);

    void render(const std::vector<Block>& blocks) override;

    void set_gnuplot_warning(bool gnuplot);

    void set_sparse_chart(bool s);

    bool gnuplot_failed();

private:
    double predict(const std::vector<Eigen::VectorXd>& solutions, int irow,
                   int icol, int width, double row, double col) const;

private:
    std::string wdir;
    std::string fname;
    std::string gnuplot_binary;

    // grid used to fit local polynomial
    size_t grid_x_coarse;
    size_t grid_y_coarse;

    // grid used to interpolate polynomial at fine scale for image output
    size_t grid_x_fine;
    size_t grid_y_fine;

    const cv::Mat& img;

    bool lpmm_mode;
    double pixel_size;

    bool gnuplot_failure;
    bool gnuplot_warning;

    double m_lower;
    double m_upper;

    int gnuplot_width;
    double zscale = 0.0;
    bool sparse_chart = false;
    double surface_max = -1;
    int mtf_contrast;
};
