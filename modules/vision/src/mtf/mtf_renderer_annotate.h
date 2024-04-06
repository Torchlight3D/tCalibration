#pragma once

#include "mtf_renderer.h"

class Mtf_renderer_annotate : public Mtf_renderer
{
public:
    Mtf_renderer_annotate(const cv::Mat& in_img, const std::string& fname,
                          bool lpmm_mode, double pixel_size, bool jpeg_output);

    void render(const std::vector<Block>& blocks) override;

    void write_number(cv::Mat& img, int px, int py, double val, double quality,
                      double font_scale);

public:
    const cv::Mat& img;
    cv::Mat out_img;
    std::string ofname;
    bool lpmm_mode;
    double pixel_size;
    bool jpeg_output = false;
};
