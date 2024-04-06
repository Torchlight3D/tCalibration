#pragma once

#include <array>
#include <vector>

#include <opencv2/core/mat.hpp>

class Display_profile
{
public:
    Display_profile();
    Display_profile(const std::vector<double>& gparm,
                    const std::vector<double>& luminance_weights);
    Display_profile(const std::vector<std::pair<uint16_t, uint16_t>>& gtable,
                    const std::vector<double>& luminance_weights);

    void force_linear();
    void force_sRGB();
    cv::Mat to_luminance(const cv::Mat& img);
    std::vector<cv::Mat> to_linear_rgb(const cv::Mat& img);

private:
    void render_parametric(const std::vector<double>& gparm);

private:
    std::array<uint16_t, 65536> lut;
    std::vector<double> luminance_weights;
    bool is_linear = false;
};
