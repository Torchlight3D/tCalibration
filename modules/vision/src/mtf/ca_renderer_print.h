#pragma once

#include "mtf_renderer.h"

class Ca_renderer_print : public Mtf_renderer
{
public:
    Ca_renderer_print(const std::string& fname, const cv::Mat& img,
                      bool allow_all_edges);

    void render(const std::vector<Block>& blocks) override;

public:
    std::string ofname;
    cv::Point2d img_centre;
    bool allow_all_edges;
};
