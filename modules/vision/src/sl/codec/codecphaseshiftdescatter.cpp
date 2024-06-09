#include "codecphaseshiftdescatter.h"

#include <opencv2/imgproc.hpp>

#include <tCore/Math>

#include "pstools.h"

namespace tl {

namespace {
constexpr unsigned int nPhases = 16;
}

// Encoder

EncoderPhaseShiftDescatter::EncoderPhaseShiftDescatter(unsigned int _screenCols,
                                                       unsigned int _screenRows,
                                                       CodecDir _dir)
    : Encoder(_screenCols, _screenRows, _dir)
{
    N = 6;
    patterns.reserve(N);

    // Precompute encoded patterns

    // Horizontally encoding patterns
    for (unsigned int i = 0; i < 3; i++) {
        float phase = 2.0 * pi_f / 3.0 * i;
        float pitch = (float)screenCols / (float)nPhases;
        cv::Mat patternI(screenRows, screenCols, CV_8UC3);

        for (unsigned int r = 0; r < screenRows; r++) {
            for (unsigned int c = 0; c < screenCols; c++) {
                float amp = 0.5 + 0.5 * cos(2 * pi_f * c / pitch - phase) *
                                      cos(2 * pi_f * r / 200 - phase);

                patternI.at<cv::Vec3b>(r, c) =
                    cv::Vec3b(255.0 * amp, 255.0 * amp, 255.0 * amp);
            }
        }

        patternI = patternI;
        patterns.push_back(patternI);
    }

    // Phase cue patterns
    for (unsigned int i = 0; i < 3; i++) {
        float phase = 2.0 * pi_f / 3.0 * i;
        float pitch = screenCols;
        cv::Mat pattern;
        pstools::calcPhaseVector(screenCols, 1, phase, pitch, pattern);
        patterns.push_back(pattern);
    }
}

cv::Mat EncoderPhaseShiftDescatter::getEncodingPattern(unsigned int depth) const
{
    return patterns[depth];
}

// Decoder
DecoderPhaseShiftDescatter::DecoderPhaseShiftDescatter(unsigned int _screenCols,
                                                       unsigned int _screenRows,
                                                       CodecDir _dir)
    : Decoder(_screenCols, _screenRows, _dir)
{
    N = 6;

    _frames.resize(N);
}

void DecoderPhaseShiftDescatter::setFrame(unsigned int depth, cv::Mat frame)
{
    _frames[depth] = frame;
}

static cv::Mat unwrap(const cv::Mat up, const cv::Mat upCue,
                      unsigned int nPhases)
{
    // Determine number of jumps
    cv::Mat P = (upCue * nPhases - up) / (2 * pi_f);

    // Round to integers
    P.convertTo(P, CV_8U);
    P.convertTo(P, CV_32F);

    // Add to phase
    cv::Mat upUnwrapped = up + P * 2 * pi_f;

    // Scale to range [0; 2pi]
    upUnwrapped *= 1.0 / nPhases;

    return upUnwrapped;
}

void DecoderPhaseShiftDescatter::decodeFrames(cv::Mat &up, cv::Mat &vp,
                                              cv::Mat &mask,
                                              cv::Mat &shading) const
{
    const std::vector<cv::Mat> frames(_frames.begin(), _frames.begin() + 3);
    pstools::phaseFromThreeFrames(frames[0], up);

    const std::vector<cv::Mat> framesCue(_frames.begin() + 3,
                                         _frames.begin() + 6);
    cv::Mat upCue;
    pstools::phaseFromThreeFrames(framesCue, upCue);

    up = unwrap(up, upCue, nPhases);
    up *= screenCols / (2 * pi_f);

    // cv::GaussianBlur(up, up, cv::Size(0,0), 3, 3);

    // Calculate modulation
    pstools::magnitudeFromThreeFrames(_frames, shading);

    // Threshold modulation image for mask
    mask = shading > 25;
    // cv::Mat edges;
    // cv::Sobel(up, edges, -1, 1, 1, 7);
    // edges = abs(edges) < 200;

    // cv::Mat strel =
    //     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    // cv::dilate(edges, edges, strel);
    // strel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6));
    // cv::erode(edges, edges, cv::Mat());

    cv::Mat dx, dy;
    cv::Sobel(up, dx, -1, 1, 0, 3);
    cv::Sobel(up, dy, -1, 0, 1, 3);
    cv::Mat edges;
    cv::magnitude(dx, dy, edges);
    mask = mask & (edges < 200);
}

} // namespace tl
