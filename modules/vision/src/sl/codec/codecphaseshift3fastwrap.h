#pragma once

#include "codec.h"

class EncoderPhaseShift3FastWrap : public Encoder
{
public:
    EncoderPhaseShift3FastWrap(unsigned int _screenCols,
                               unsigned int _screenRows, CodecDir _dir);
    // Encoding
    cv::Mat getEncodingPattern(unsigned int depth) const override;

private:
    std::vector<cv::Mat> patterns;
};

class DecoderPhaseShift3FastWrap : public Decoder
{
public:
    DecoderPhaseShift3FastWrap(unsigned int _screenCols,
                               unsigned int _screenRows, CodecDir _dir);
    // Decoding
    void setFrame(unsigned int depth, cv::Mat frame) override;
    void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask,
                      cv::Mat &shading) const override;

private:
    std::vector<cv::Mat> frames;
};
