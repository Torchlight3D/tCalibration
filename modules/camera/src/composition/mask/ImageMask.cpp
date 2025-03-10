﻿#include "ImageMask.h"

namespace tl {

ImageMask::ImageMask() : _mask(cv::Mat(0, 0, CV_8UC1)), _scale(1.0){};

ImageMask::ImageMask(const cv::Mat& mask, double scale)
{
    //    SM_ASSERT_EQ(std::runtime_error, mask.type(), CV_8UC1,
    //                 "The mask must be 8-bit one channel");
    _mask = mask.clone();
    _scale = scale;
}

ImageMask::~ImageMask() {}

void ImageMask::setMask(const cv::Mat& mask)
{
    //    SM_ASSERT_EQ(std::runtime_error, mask.type(), CV_8UC1,
    //                 "The mask must be 8-bit one channel");
    _mask = mask.clone();
}

bool ImageMask::isSet() const { return _mask.data; }

const cv::Mat& ImageMask::mask() const { return _mask; }

void ImageMask::setScale(double scale) { _scale = scale; }

double ImageMask::scale() const { return _scale; }

void ImageMask::setMaskFromMatrix(const Eigen::MatrixXi& mask)
{
    _mask.create(mask.rows(), mask.cols(), CV_8UC1);
    for (int r = 0; r < mask.rows(); ++r)
    {
        uchar* p = _mask.ptr(r);
        for (int c = 0; c < mask.cols(); ++c, ++p)
        {
            *p = mask(r, c);
        }
    }
}

Eigen::MatrixXi ImageMask::getMaskAsMatrix() const
{
    Eigen::MatrixXi rval(_mask.rows, _mask.cols);

    for (int r = 0; r < rval.rows(); ++r)
    {
        const uchar* p = _mask.ptr(r);
        for (int c = 0; c < rval.cols(); ++c, ++p)
        {
            rval(r, c) = *p;
        }
    }
    return rval;
}

} // namespace tl
