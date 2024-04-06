#pragma once

#include "ellipse.h"

class Ellipse_decoder
{
public:
    Ellipse_decoder(const Ellipse_detector &e, const cv::Mat &img);

    void extract_code(const Ellipse_detector &e, const cv::Mat &img);

public:
    int code;
    bool valid;
    double ratio;
};
