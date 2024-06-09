#pragma once

#include "codec.h"

namespace tl {

class EncoderPhaseShift4 : public Encoder
{
public:
    EncoderPhaseShift4(unsigned int _screenCols, unsigned int _screenRows,
                       CodecDir _dir);

    cv::Mat getEncodingPattern(unsigned int depth) const override;

private:
    std::vector<cv::Mat> patterns;
};

class DecoderPhaseShift4 : public Decoder
{
public:
    DecoderPhaseShift4(unsigned int _screenCols, unsigned int _screenRows,
                       CodecDir _dir);

    void setFrame(unsigned int depth, cv::Mat frame) override;
    void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask,
                      cv::Mat &shading) const override;

private:
    std::vector<cv::Mat> frames;
};

} // namespace tl
