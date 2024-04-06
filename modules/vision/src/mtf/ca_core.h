#pragma once

#include "mtf_core.h"

class Ca_core
{
public:
    Ca_core(Mtf_core& mtf_core) : mtf_core(mtf_core) {}

    void set_rgb_channels(std::vector<cv::Mat> in_channels);

    void calculate_ca(Block& block);
    void set_allow_all_edges();

public:
    Mtf_core& mtf_core;
    std::vector<cv::Mat> channels;

private:
    void extract_rgb_lsf_bayer(Block& block, const cv::Mat& img,
                               const cv::Mat& bayer_img,
                               std::vector<std::vector<double>>& red_lsf,
                               std::vector<std::vector<double>>& green_lsf,
                               std::vector<std::vector<double>>& blue_lsf);

    void extract_rgb_lsf(Block& block, const cv::Mat& img,
                         const std::vector<cv::Mat>& channels,
                         std::vector<std::vector<double>>& red_lsf,
                         std::vector<std::vector<double>>& green_lsf,
                         std::vector<std::vector<double>>& blue_lsf);

private:
    bool allow_all_edges = false;
};
