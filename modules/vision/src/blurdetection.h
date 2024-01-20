#pragma once

namespace cv {
class Mat;
}

namespace tl {

// TODO
// 1. Normalize input image so that the evaluation value makes sense
// 2. Image type (depth)

class BlurDetection
{
public:
    virtual double evaluate(const cv::Mat& image) = 0;
};

// WARNING: Not ready
class DomainEntropyBlurDetection final : public BlurDetection
{
public:
    double evaluate(const cv::Mat& image) override;
};

class TenegradBlurDetection final : public BlurDetection
{
public:
    double evaluate(const cv::Mat& image) override;
};

class LaplacianBlurDetection final : public BlurDetection
{
public:
    double evaluate(const cv::Mat& image) override;
};

class LaplacianModifiedBlurDetection final : public BlurDetection
{
public:
    double evaluate(const cv::Mat& image) override;
};

class VarianceBlurDetection final : public BlurDetection
{
public:
    double evaluate(const cv::Mat& image) override;
};

} // namespace tl
