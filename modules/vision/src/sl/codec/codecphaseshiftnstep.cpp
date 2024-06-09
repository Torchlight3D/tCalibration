#include "codecphaseshiftnstep.h"

#include <opencv2/imgproc.hpp>

#include <tCore/Math>

#include "pstools.h"

namespace tl {
namespace {
constexpr unsigned int nPhases = 24;
constexpr unsigned int nSteps = 9;
} // namespace

// Encoder
EncoderPhaseShiftNStep::EncoderPhaseShiftNStep(unsigned int _screenCols,
                                               unsigned int _screenRows,
                                               CodecDir _dir)
    : Encoder(_screenCols, _screenRows, _dir)
{
    // Set N
    N = nSteps + 3;
    if (dir == CodecDirBoth)
        N *= 2;
    patterns.reserve(N);

    // Precompute encoded patterns

    if (dir & CodecDirHorizontal) {
        // Horizontally encoding patterns
        for (unsigned int i = 0; i < nSteps; i++) {
            float phase = 2.0 * pi_f / nSteps * i;
            float pitch = (float)screenCols / (float)nPhases;
            cv::Mat patternI;
            pstools::calcPhaseVector(screenCols, 1, phase, pitch, patternI);
            patterns.push_back(patternI);
        }

        // Phase cue patterns
        for (unsigned int i = 0; i < 3; i++) {
            float phase = 2.0 * pi_f / 3.0 * i;
            float pitch = screenCols;
            cv::Mat patternI;
            pstools::calcPhaseVector(screenCols, 1, phase, pitch, patternI);
            patterns.push_back(patternI);
        }
    }

    if (dir & CodecDirVertical) {
        // Precompute vertically encoding patterns
        for (unsigned int i = 0; i < nSteps; i++) {
            float phase = 2.0 * pi_f / nSteps * i;
            float pitch = (float)screenRows / (float)nPhases;
            cv::Mat patternI;
            pstools::calcPhaseVector(screenRows, 1, phase, pitch, patternI);
            patterns.push_back(patternI);
        }

        // Precompute vertically phase cue patterns
        for (unsigned int i = 0; i < 3; i++) {
            float phase = 2.0 * pi_f / 3.0 * i;
            float pitch = screenRows;
            cv::Mat patternI;
            pstools::calcPhaseVector(screenRows, 1, phase, pitch, patternI);
            patterns.push_back(patternI);
        }
    }

#if 0
        for(unsigned int i=0; i<patterns.size(); i++){
            std::stringstream fileNameStream;
            fileNameStream << "pattern_" << std::setw(2) << std::setfill('0') << i << ".bmp";
            cv::imwrite(fileNameStream.str(), patterns[i]);
        }

#endif
}

cv::Mat EncoderPhaseShiftNStep::getEncodingPattern(unsigned int depth) const
{
    return patterns[depth];
}

// Decoder
DecoderPhaseShiftNStep::DecoderPhaseShiftNStep(unsigned int _screenCols,
                                               unsigned int _screenRows,
                                               CodecDir _dir)
    : Decoder(_screenCols, _screenRows, _dir)
{
    // Set N
    N = nSteps + 3;
    if (dir == CodecDirBoth)
        N *= 2;

    _frames.resize(N);
}

void DecoderPhaseShiftNStep::setFrame(unsigned int depth, cv::Mat frame)
{
    _frames[depth] = frame;
}

void DecoderPhaseShiftNStep::decodeFrames(cv::Mat &up, cv::Mat &vp,
                                          cv::Mat &mask, cv::Mat &shading) const
{
    std::vector<cv::Mat> fIcomp;

    if (dir & CodecDirHorizontal) {
        const std::vector<cv::Mat> frames{_frames.begin(),
                                          _frames.begin() + nSteps};
        fIcomp = pstools::getDFTComponents(frames);
        cv::phase(fIcomp[2], -fIcomp[3], up);

        const std::vector<cv::Mat> framesCue{_frames.begin() + nSteps,
                                             _frames.begin() + nSteps + 3};
        cv::Mat upCue;
        pstools::phaseFromThreeFrames(framesCue, upCue);

        up = pstools::unwrapWithCue(up, upCue, nPhases);
        up *= screenCols / (2 * pi_f);

        // cv::GaussianBlur(up, up, cv::Size(0,0), 1, 1);
    }

    if (dir & CodecDirVertical) {
        const std::vector<cv::Mat> frames(_frames.end() - nSteps - 3,
                                          _frames.end() - 3);

        fIcomp = pstools::getDFTComponents(frames);
        cv::phase(fIcomp[2], -fIcomp[3], vp);

        const std::vector<cv::Mat> framesCue(_frames.end() - 3, _frames.end());
        cv::Mat vpCue;
        pstools::phaseFromThreeFrames(framesCue, vpCue);

        vp = pstools::unwrapWithCue(vp, vpCue, nPhases);
        vp *= screenCols / (2 * pi_f);
    }

    {
        const std::vector<cv::Mat> frames(_frames.begin(),
                                          _frames.begin() + nSteps);
        fIcomp = pstools::getDFTComponents(frames);
        cv::magnitude(fIcomp[2], -fIcomp[3], shading);

        shading.convertTo(shading, CV_8U, 2.0 / nSteps);
    }

    // Threshold on energies
    mask = shading > 20;

    // Threshold on gradient of phase
    // cv::Mat edges;
    // cv::Sobel(up, edges, -1, 1, 1, 7);
    // edges = abs(edges) < 500;
    // cv::Mat strel =
    //     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    // cv::erode(edges, edges, strel);
    // mask = mask & edges;
}

} // namespace tl
