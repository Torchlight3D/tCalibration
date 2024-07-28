#include "srgbrender.h"

#include <opencv2/core/core.hpp>

cv::Mat Srgb_render::linear_to_sRGB(const cv::Mat& img)
{
    // quantize the range [0, 65535] to [0, 8191] to keep histogram smaller
    std::vector<float> histo(8194, 0.0);
    const uint16_t* p = (uint16_t*)img.data;
    const uint16_t* sentinel = p + img.total();
    while (p < sentinel) {
        histo[*p >> 3]++;
        p += 1;
    }

    size_t maximum = histo.size() - 1;
    while (maximum > 0 && histo[maximum] == 0) {
        maximum--;
    }

    size_t minimum = 0;
    while (minimum < histo.size() - 1 && histo[minimum] == 0) {
        minimum++;
    }

    // compute 2% and 98% cut-off points
    float csum = histo[maximum];
    size_t maxval = maximum;
    while (maxval > 0 && csum < 0.02 * img.total()) {
        maxval--;
        csum += histo[maxval];
    }

    csum = histo[minimum];
    size_t minval = minimum;
    while (minval < histo.size() - 1 && csum < 0.02 * img.total()) {
        minval++;
        csum += histo[minval];
    }

    if (fabs(double(maxval - minval)) < 0.05 * (maximum - minimum)) {
        // we are probably looking at a synthetic image with insufficient dark
        // areas to make the 2% threshold meaningful just go with the min/max
        minval = minimum;
        maxval = maximum;
    }

    minval *= 8;
    maxval *= 8;

    // scale intenisty so that minval -> 0 and maxval -> 255
    double alpha = 1;
    double beta = 0;
    if (maxval > minval) {
        alpha = 1.0 / double(maxval - minval);
        beta = -alpha * minval;
    }

    // convert values to sRGB gamma using a quantized lut
    std::vector<uint8_t> lut(8192, 0);
    for (size_t i = 0; i < lut.size(); i++) {
        double sv = std::max(0.0, std::min(1.0, double(i << 3) * alpha + beta));
        lut[i] = reverse_gamma(sv);
    }

    cv::Mat temp_img(img.rows, img.cols, CV_8UC1);
    uint8_t* d = temp_img.data;
    p = (uint16_t*)img.data;
    while (p < sentinel) {
        *d++ = lut[*p++ >> 3];
    }
    cv::Mat out_img;
    cv::merge(std::vector<cv::Mat>(3, temp_img), out_img);

    return out_img;
}

unsigned char Srgb_render::reverse_gamma(double x)
{
    // x in [0, 1]
    constexpr double C_linear = 0.0031308;
    constexpr double S_linear = 12.9232102;
    constexpr double SRGB_a = 0.055;

    if (x < C_linear) {
        return lrint(255 * x * S_linear);
    }
    return lrint(255 * ((1 + SRGB_a) * pow(x, 1.0 / 2.4) - SRGB_a));
}
