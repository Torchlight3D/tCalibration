#pragma once

#include "bayer.h"

class Job_metadata
{
public:
    Job_metadata() {}
    Job_metadata(double pixel_pitch, double mtf_contrast, Bayer::bayer_t bayer)
        : pixel_pitch(pixel_pitch), mtf_contrast(mtf_contrast), bayer(bayer)
    {
    }

    void set_image_dims(uint32_t width, uint32_t height)
    {
        image_width = width;
        image_height = height;
    }

public:
    double pixel_pitch = 1;
    double mtf_contrast = 0.5;
    Bayer::bayer_t bayer = Bayer::bayer_t::NONE;
    int channels = 1;
    uint32_t image_width = 0;
    uint32_t image_height = 0;
};
