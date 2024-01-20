#pragma once

#include <Eigen/Core>
#include <opencv2/core/mat.hpp>

namespace tl {

class ImageMask
{
public:
    ImageMask();
    explicit ImageMask(const cv::Mat& mask, double scale = 1.0);
    ~ImageMask();

    void setScale(double scale);
    double scale() const;

    void setMask(const cv::Mat& mask);
    const cv::Mat& mask() const;
    bool isSet() const;

    // These guys mainly support the python interface
    void setMaskFromMatrix(const Eigen::MatrixXi& mask);
    Eigen::MatrixXi getMaskAsMatrix() const;

    template <typename DERIVED>
    bool isValid(const Eigen::MatrixBase<DERIVED>& k) const
    {
        int k1 = k(1, 0) * _scale;
        int k0 = k(0, 0) * _scale;
        // \todo fix this when it is initialized properly
        return !_mask.data || (_mask.at<unsigned char>(k1, k0) > 0);
        // return true;
    }

private:
    cv::Mat _mask;
    double _scale;
};

} // namespace tl
