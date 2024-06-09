#include "codecphaseshift2x3.h"

#include <opencv2/imgproc.hpp>

#include <tCore/Math>

#include "pstools.h"

namespace tl {

namespace {
constexpr unsigned int kNumPhases = 16;
}

// Encoder

EncoderPhaseShift2x3::EncoderPhaseShift2x3(unsigned int _screenCols,
                                           unsigned int _screenRows,
                                           CodecDir _dir)
    : Encoder(_screenCols, _screenRows, _dir)
{
    if (dir == CodecDirBoth)
        N = 12;
    else
        N = 6;

    patterns.reserve(N);

    // Precompute encoded patterns
    if (dir & CodecDirHorizontal) {
        // Horizontally encoding patterns
        for (unsigned int i = 0; i < 3; i++) {
            float phase = 2.0 * pi_f / 3.0 * i;
            float pitch = (float)screenCols / (float)kNumPhases;
            cv::Mat pattern;
            pstools::calcPhaseVector(screenCols, 1, phase, pitch, pattern);
            patterns.push_back(pattern);
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
    if (dir & CodecDirVertical) {
        // Precompute vertically encoding patterns
        for (unsigned int i = 0; i < 3; i++) {
            float phase = 2.0 * pi_f / 3.0 * i;
            float pitch = (float)screenRows / (float)kNumPhases;
            cv::Mat pattern;
            pstools::calcPhaseVector(screenRows, 0, phase, pitch, pattern);
            patterns.push_back(pattern);
        }

        // Precompute vertically phase cue patterns
        for (unsigned int i = 0; i < 3; i++) {
            float phase = 2.0 * pi_f / 3.0 * i;
            float pitch = screenRows;
            cv::Mat pattern;
            pstools::calcPhaseVector(screenRows, 0, phase, pitch, pattern);
            patterns.push_back(pattern);
        }
    }
}

cv::Mat EncoderPhaseShift2x3::getEncodingPattern(unsigned int depth) const
{
    return patterns[depth];
}

// Decoder
DecoderPhaseShift2x3::DecoderPhaseShift2x3(unsigned int _screenCols,
                                           unsigned int _screenRows,
                                           CodecDir _dir)
    : Decoder(_screenCols, _screenRows, _dir)
{
    if (dir == CodecDirBoth)
        N = 12;
    else
        N = 6;

    _frames.resize(N);
}

void DecoderPhaseShift2x3::setFrame(unsigned int depth, cv::Mat frame)
{
    _frames[depth] = frame;
}

void DecoderPhaseShift2x3::decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask,
                                        cv::Mat &shading) const
{
    if (dir & CodecDirHorizontal) {
        const std::vector<cv::Mat> frames(_frames.begin(), _frames.begin() + 3);
        pstools::phaseFromThreeFrames(frames, up);

        const std::vector<cv::Mat> framesCue(_frames.begin() + 3,
                                             _frames.begin() + 6);
        cv::Mat upCue;
        pstools::phaseFromThreeFrames(framesCue, upCue);

        up = pstools::unwrapWithCue(up, upCue, kNumPhases);
        up *= screenCols / (2 * pi_f);

        // cv::GaussianBlur(up, up, cv::Size(0,0), 3, 3);
    }

    if (dir & CodecDirVertical) {
        const std::vector<cv::Mat> frames(_frames.end() - 6, _frames.end() - 3);
        pstools::phaseFromThreeFrames(frames, vp);

        const std::vector<cv::Mat> framesCue(_frames.end() - 3, _frames.end());
        cv::Mat vpCue;
        pstools::phaseFromThreeFrames(framesCue, vpCue);

        vp = pstools::unwrapWithCue(vp, vpCue, kNumPhases);
        vp *= screenRows / (2 * pi_f);
    }

    // Calculate modulation
    pstools::magnitudeFromThreeFrames(_frames, shading);

    //  Threshold modulation image for mask
    mask = shading > 55;
    // cv::Mat edges;
    // cv::Sobel(up, edges, -1, 1, 1, 7);
    // edges = abs(edges) < 200;

    // cv::Mat strel =
    //     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    // cv::dilate(edges, edges, strel);
    // strel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6));
    // cv::erode(edges, edges, cv::Mat());

    // cv::Mat dx, dy;
    // cv::Sobel(up, dx, -1, 1, 0, 3);
    // cv::Sobel(up, dy, -1, 0, 1, 3);
    // cv::Mat edges;
    // cv::magnitude(dx, dy, edges);
    // mask = mask & (edges < 200);
}

} // namespace tl
