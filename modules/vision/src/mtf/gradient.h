#pragma once

#include <opencv2/core/mat.hpp>

class Gradient
{
public:
    Gradient(const cv::Mat& in_img);
    virtual ~Gradient();

    void write_grad_x(int x, int y, float val);
    const cv::Mat& grad_x() const;
    const cv::Mat& grad_y() const;

    void write_grad_y(int x, int y, float val);
    float grad_x(int x, int y) const;
    float grad_y(int x, int y) const;

    float grad_magnitude(int x, int y) const;

    int width() const;
    int height() const;

    void release();

private:
    void _compute_gradients(const cv::Mat& smoothed_im);

protected:
    int _width;
    int _height;

    cv::Mat _gradient_x;
    cv::Mat _gradient_y;
};
