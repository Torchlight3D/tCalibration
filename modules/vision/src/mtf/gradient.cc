#include "gradient.h"

#include <opencv2/imgproc.hpp>

#include "common_types.h"

Gradient::Gradient(const cv::Mat& in_img)
    : _width(in_img.cols), _height(in_img.rows)
{
    cv::Mat in_float;
    double min_val = 0;
    double max_val = 0;
    minMaxLoc(in_img, &min_val, &max_val);
    in_img.convertTo(in_float, CV_32FC1, 1.0 / max_val);

    cv::Mat smoothed;
    cv::GaussianBlur(in_float, smoothed, cv::Size(5, 5), 1.2, 1.2);

    _compute_gradients(smoothed);
    smoothed.release();
}

Gradient::~Gradient() {}

const cv::Mat& Gradient::grad_x() const { return _gradient_x; }

void Gradient::write_grad_x(int x, int y, float val)
{
    _gradient_x.at<float>(y, x) = val;
}

const cv::Mat& Gradient::grad_y() const { return _gradient_y; }

void Gradient::write_grad_y(int x, int y, float val)
{
    _gradient_y.at<float>(y, x) = val;
}

float Gradient::grad_x(int x, int y) const
{
    return _gradient_x.at<float>(y, x);
}

float Gradient::grad_y(int x, int y) const
{
    return _gradient_y.at<float>(y, x);
}

float Gradient::grad_magnitude(int x, int y) const
{
    size_t i = _gradient_x.cols * y + x;
    return SQR(*((float*)_gradient_x.data + i)) +
           SQR(*((float*)_gradient_y.data + i));
}

int Gradient::width() const { return _width; }

int Gradient::height() const { return _height; }

void Gradient::release()
{
    _gradient_x.release();
    _gradient_y.release();
}

void Gradient::_compute_gradients(const cv::Mat& smoothed_im)
{
    _gradient_x = cv::Mat(smoothed_im.rows, smoothed_im.cols, CV_32FC1);
    _gradient_y = cv::Mat(smoothed_im.rows, smoothed_im.cols, CV_32FC1);

    float* gxp = (float*)_gradient_x.data + 1;
    float* smp = (float*)smoothed_im.data + 1;
    for (int i = 0; i < smoothed_im.rows * smoothed_im.cols - 2;
         i++) { // skip first and last pixels
        *gxp = *(smp + 1) - *(smp - 1);
        gxp++;
        smp++;
    }
    for (int r = 0; r < smoothed_im.rows; r++) {
        _gradient_x.ptr<float>(r)[0] = 0;
        _gradient_x.ptr<float>(r)[smoothed_im.cols - 1] = 0;
    }
    float* gyp = (float*)_gradient_y.data + _gradient_y.cols;
    smp = (float*)smoothed_im.data + smoothed_im.cols;
    for (int i = 0;
         i < smoothed_im.rows * smoothed_im.cols - 2 * smoothed_im.cols;
         i++) { // skip first and last row
        *gyp = *(smp + smoothed_im.cols) - *(smp - smoothed_im.cols);
        gyp++;
        smp++;
    }
    for (int c = 0; c < smoothed_im.cols; c++) {
        _gradient_y.ptr<float>(0)[c] = 0;
        _gradient_y.ptr<float>(smoothed_im.rows - 1)[c] = 0;
    }
}
