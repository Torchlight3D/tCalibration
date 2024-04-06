#pragma once

#include "mtf_renderer.h"

class Mtf_renderer_blocks : public Mtf_renderer
{
public:
    Mtf_renderer_blocks(const cv::Mat& in_img, const std::string& fname);

    void render(const std::vector<Block>& blocks) override;

public:
    const cv::Mat& img;
    cv::Mat out_img;
    std::string ofname;
};
