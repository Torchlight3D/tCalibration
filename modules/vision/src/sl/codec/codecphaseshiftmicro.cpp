#include "codecphaseshiftmicro.h"

#include <opencv2/imgproc.hpp>

#include <tMath/MathBase>

#include "cvtools.h"
#include "pstools.h"

namespace {
constexpr unsigned int F = 10;
// constexpr float frequencies[] = {32.54f, 30.f,   30.44f, 30.68f,
//                                  34.04f, 34.34f, 35.99f};
// constexpr float frequencies[] = {74.f, 70.f, 71.f, 72.f, 73.f,
//                                  75.f, 76.f, 77.f, 78.f, 79.f};
constexpr float frequencies[] = {75.02f, 70.f,   71.32f, 72.47f, 73.72f,
                                 76.23f, 77.35f, 78.4f,  79.22f, 80.f};
} // namespace

// Encoder
EncoderPhaseShiftMicro::EncoderPhaseShiftMicro(unsigned int _screenCols,
                                               unsigned int _screenRows,
                                               CodecDir _dir)
    : Encoder(_screenCols, _screenRows, _dir)
{
    // Set N
    N = F + 2;

    // Precompute encoded patterns

    // Main frequency encoding patterns
    for (unsigned int i = 0; i < 3; i++) {
        float phase = -2.0 * pi_f / 3.0 * i;
        float pitch = frequencies[0];
        cv::Mat patternI(1, 1, CV_8U);
        patternI = pstools::computePhaseVector(screenCols, phase, pitch);
        patternI = patternI.t();
        patterns.push_back(patternI);
    }

    // Additional frequency patterns
    for (unsigned int i = 1; i < F; i++) {
        float phase = 0.0;
        if (i % 2)
            phase = pi_f;
        float pitch = frequencies[i];
        cv::Mat patternI;
        patternI = pstools::computePhaseVector(screenCols, phase, pitch);
        patternI = patternI.t();
        patterns.push_back(patternI);
    }

#if 0
        for(unsigned int i=0; i<patterns.size(); i++){
            std::stringstream fileNameStream;
            fileNameStream << "pattern_" << std::setw(2) << std::setfill('0') << i << ".bmp";
            cv::imwrite(fileNameStream.str(), patterns[i]);
        }

#endif
}

cv::Mat EncoderPhaseShiftMicro::getEncodingPattern(unsigned int depth) const
{
    return patterns[depth];
}

// Decoder
DecoderPhaseShiftMicro::DecoderPhaseShiftMicro(unsigned int _screenCols,
                                               unsigned int _screenRows,
                                               CodecDir _dir)
    : Decoder(_screenCols, _screenRows, _dir)
{
    N = F + 2;

    frames.resize(N);
}

void DecoderPhaseShiftMicro::setFrame(unsigned int depth, cv::Mat frame)
{
    frames[depth] = frame;
}

void DecoderPhaseShiftMicro::decodeFrames(cv::Mat &up, cv::Mat &vp,
                                          cv::Mat &mask, cv::Mat &shading) const
{
    //    std::vector<cv::Mat> framesMain(frames.begin(), frames.begin()+3);

    //    // Horizontal decoding
    //    up = pstools::getPhase(framesMain[0], framesMain[1], framesMain[2]);
    //    shading = pstools::getMagnitude(framesMain[0], framesMain[1],
    //    framesMain[2]);

    //    mask = shading > 25;

    int rows = frames[0].rows;
    int cols = frames[0].cols;

    // Construct system of equations
    cv::Mat Mmicro(F + 2, F + 2, CV_32F, cv::Scalar(0.0));
    Mmicro.at<float>(0, 1) = 1.0;
    Mmicro.at<float>(0, 2) = 0.0;
    Mmicro.at<float>(1, 1) = cos(2.0 * pi_f / 3.0);
    Mmicro.at<float>(1, 2) = -sin(2.0 * pi_f / 3.0);
    Mmicro.at<float>(2, 1) = cos(4.0 * pi_f / 3.0);
    Mmicro.at<float>(2, 2) = -sin(4.0 * pi_f / 3.0);
    Mmicro.col(0).setTo(1.0);
    for (unsigned int i = 0; i < F - 1; i++)
        Mmicro.at<float>(3 + i, 3 + i) = (i % 2) * 2 - 1;

    cv::Mat Rmicro(F + 2, rows * cols, CV_32F);
    for (unsigned int i = 0; i < F + 2; i++) {
        frames[i].reshape(0, 1).copyTo(Rmicro.row(i));
    }

    // Solve
    cv::Mat Ufact;
    cv::solve(Mmicro, Rmicro, Ufact);

    // Shading
    cv::Mat amp;
    cv::magnitude(Ufact.row(1), Ufact.row(2), amp);
    shading = amp.reshape(0, rows);
    shading.convertTo(shading, CV_8U, 2.0);
    mask = shading > 20;

    cv::Mat CosSin(F + 1, rows * cols, CV_32F);
    for (unsigned int i = 0; i < F + 1; i++) {
        cv::divide(Ufact.row(i + 1), amp, CosSin.row(i));
    }

    //    cvtools::writeMat(CosSin, "CosSin.mat", "CosSin");

    // Reference CosSin values
    cv::Mat RefCosSin(F + 1, screenCols, CV_32F);
    for (unsigned int i = 0; i < screenCols; i++) {
        RefCosSin.at<float>(0, i) = std::cos(2 * pi_f * i / frequencies[0]);
        RefCosSin.at<float>(1, i) = std::sin(2 * pi_f * i / frequencies[0]);
        for (unsigned int j = 2; j < F + 1; j++) {
            RefCosSin.at<float>(j, i) =
                std::cos(2 * pi_f * i / frequencies[j - 1]);
        }
    }

    //    cvtools::writeMat(RefCosSin, "RefCosSin.mat", "RefCosSin");

    // Find best match value
    cv::Mat upCueMatch(1, rows * cols, CV_32F);
    cv::Mat bestDistMatch(1, rows * cols, CV_32F);
    for (int i = 0; i < rows * cols; i++) {
        int bestMatch = -1;
        float bestDist = INFINITY;
        for (unsigned int j = 0; j < screenCols; j++) {
            //            float dist = cv::norm(CosSin.col(i) -
            //            RefCosSin.col(j), cv::NORM_L2SQR);
            float dist = 0.0;
            for (unsigned int k = 0; k < F + 1; k++) {
                float diff = CosSin.at<float>(k, i) - RefCosSin.at<float>(k, j);
                dist += diff * diff;
            }
            if (dist < bestDist) {
                bestDist = dist;
                bestMatch = j;
            }
        }
        upCueMatch.at<float>(0, i) = bestMatch;
        bestDistMatch.at<float>(0, i) = bestDist;
    }

    cv::Mat upCue = upCueMatch.reshape(0, rows);

    bestDistMatch = bestDistMatch.reshape(0, rows);
    cvtools::writeMat(bestDistMatch, "bestDistMatch.mat", "bestDistMatch");

    upCue *= (2 * pi_f) / screenCols;
    up = pstools::getPhase(frames[0], frames[2], frames[1]);
    up = pstools::unwrapWithCue(up, upCue, screenCols / frequencies[0]);
    up *= screenCols / (2 * pi_f);
    // up = upCue;
    //     cvtools::writeMat(up, "up.mat", "up");
}
