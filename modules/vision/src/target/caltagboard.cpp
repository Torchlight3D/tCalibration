#include "caltagboard.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <tCore/Math>

namespace tl {

bool checkBitCode(int code, cv::Point2f &pt)
{
    // #define USE4x4CALTAGTARGET
#ifdef USE4x4CALTAGTARGET
    static const int realBitCodes[4][4] = {{55832, 36200, 48172, 36446},
                                           {52024, 40282, 44062, 40830},
                                           {48648, 35962, 48446, 53085},
                                           {44840, 56409, 60701, 56957}};

    for (int i = 0; i < 4; i++) {
        for (int j = 3; j >= 0; j--) {
            if (code == realBitCodes[i][j]) {
                *pt = cv::Point2f((float)(i - 7), (float)(j - 7));
                return (true);
            }
        }
    }
#else

// #define USESQUARECALTAGTARGET
#ifdef USESQUARECALTAGTARGET
    static const int realBitCodes[15][15] = {
        {8578, 12720, 56439, 52567, 56677, 52293, 40038, 36166, 40308, 63606,
         59734, 63844, 59460, 47207, 43335},
        {47477, 65079, 61207, 60933, 48678, 48948, 55862, 51990, 56100, 51716,
         39463, 35591, 39733, 35349, 53107},
        {56915, 57153, 36706, 40514, 36464, 40784, 60274, 64082, 60000, 43875,
         47683, 43633, 47953, 60723, 64531},
        {60449, 64769, 44322, 48130, 44080, 51506, 55314, 55552, 35107, 38915,
         34865, 39185, 23879, 19829, 3190},
        {7510, 3428, 7236, 31046, 26996, 30804, 10359, 14679, 14405, 28199,
         32519, 28469, 32277, 11830, 16150},
        {12068, 15876, 18982, 23302, 19252, 23060, 6935, 2853, 6661, 24419,
         24177, 20305, 8050, 3666, 7776},
        {31586, 27202, 31344, 27472, 15219, 10835, 14945, 11073, 32035, 27651,
         31793, 27921, 15666, 11282, 11520},
        {22818, 22576, 18704, 6451, 6177, 54646, 50262, 54372, 50500, 38247,
         33863, 38005, 34133, 61815, 57431},
        {57669, 45414, 41030, 45172, 41300, 63286, 58902, 63012, 59140, 46887,
         42503, 46645, 42773, 54071, 49687},
        {53797, 49925, 37670, 33286, 37428, 50802, 55122, 51040, 34403, 38723,
         34673, 38481, 57971, 62291, 58209},
        {62017, 41570, 45890, 41840, 45648, 58418, 62738, 58656, 62464, 42019,
         46339, 42289, 46097, 49203, 53523},
        {49441, 53249, 37122, 33072, 17766, 21574, 17524, 21844, 5207, 1125,
         5445, 24935, 28743, 24693, 29013},
        {8566, 12374, 8292, 12612, 26406, 30214, 26164, 30484, 10039, 13847,
         9765, 14085, 17191, 20999, 16949},
        {21269, 822, 4630, 4868, 22114, 18242, 22384, 18000, 5747, 1875, 5985,
         1601, 29283, 25411, 29553},
        {25169, 12914, 9042, 13152, 29730, 25858, 30000, 25616, 13363, 9491,
         13601, 20515, 16643, 20785, 4146}};

    for (int i = 0; i < 15; i++) {
        for (int j = 14; j >= 0; j--) {
            if (code == realBitCodes[i][j]) {
                *pt = cv::Point2f((float)(i - 7), (float)(j - 7));
                return (true);
            }
        }
    }
#else
    constexpr int kRow{20};
    constexpr int kCol{14};
    static constexpr int realBitCodes[kRow][kCol]{
        {19853, 19918, 20121, 20186, 20373, 20438, 20497, 20562, 20692, 20765,
         20830, 20891, 20952, 21001},
        {18674, 18747, 18808, 18877, 18942, 18991, 19052, 19113, 19235, 19296,
         19365, 19650, 19723, 19784},
        {17633, 17704, 17771, 17838, 17901, 17980, 18047, 18106, 18224, 18291,
         18358, 18421, 18487, 18609},
        {16530, 16593, 16664, 16731, 16798, 16861, 16908, 16975, 17034, 17286,
         17349, 17444, 17511, 17570},
        {15514, 15577, 15632, 15699, 15766, 15829, 15943, 16002, 16065, 16136,
         16203, 16270, 16333, 16471},
        {14380, 14447, 14506, 14569, 14624, 14691, 14758, 14821, 14967, 15089,
         15160, 15294, 15357, 15455},
        {13375, 13436, 13497, 13562, 13680, 13749, 13814, 13863, 13924, 13985,
         14050, 14123, 14253, 14318},
        {12303, 12364, 12425, 12490, 12547, 12677, 12742, 12884, 12945, 13010,
         13083, 13144, 13213, 13278},
        {11195, 11256, 11289, 11423, 11484, 11541, 11606, 11667, 11777, 11842,
         11911, 11972, 12110, 12171},
        {10219, 10281, 10346, 10415, 10533, 10598, 10659, 10720, 10801, 10866,
         10935, 10996, 11069, 11134},
        {9179, 9274, 9337, 9404, 9471, 9526, 9589, 9648, 9715, 9825, 9892, 9959,
         10030, 10093},
        {8001, 8068, 8135, 8265, 8399, 8454, 8517, 8643, 8722, 8852, 8919, 8990,
         9053, 9112},
        {7092, 7159, 7190, 7253, 7312, 7379, 7450, 7580, 7647, 7694, 7757, 7816,
         7883, 7938},
        {6116, 6245, 6304, 6371, 6442, 6505, 6572, 6639, 6718, 6781, 6840, 6907,
         6962, 7025},
        {5173, 5238, 5299, 5360, 5433, 5498, 5567, 5628, 5677, 5742, 5803, 5864,
         5986, 6055},
        {4166, 4227, 4361, 4426, 4495, 4556, 4637, 4702, 4763, 4824, 4881, 4946,
         5015, 5076},
        {3152, 3221, 3286, 3359, 3420, 3481, 3546, 3595, 3656, 3725, 3847, 3908,
         3969, 4034},
        {2278, 2351, 2412, 2473, 2538, 2619, 2680, 2749, 2814, 2871, 2932, 2993,
         3058, 3091},
        {1340, 1407, 1466, 1529, 1576, 1643, 1710, 1773, 1828, 1895, 1954, 2017,
         2083, 2213},
        {197, 335, 394, 457, 603, 670, 733, 788, 855, 914, 977, 1139, 1206,
         1269}};

    for (int i = 0; i < kRow; i++) {
        for (int j = kCol; j >= 0; j--) {
            if (code == realBitCodes[i][j]) {
                pt = cv::Point2f(cv::Point{j - 7, i - 7});
                return true;
            }
        }
    }
#endif
#endif
    return false;
}

std::vector<cv::Point2f> sortPoints(const std::vector<cv::Point> &points)
{
    cv::Point2f center;
    for (const auto &point : points) {
        center += cv::Point2f(point);
    }

    center /= static_cast<float>(points.size());

    std::vector<float> angles;
    std::vector<size_t> indices;
    for (size_t n{0}; n < points.size(); ++n) {
        auto offset = cv::Point2f(points[n]) - center;
        angles.push_back(atan2(offset.y, offset.x));
        indices.push_back(n);
    }

    // sort
    for (size_t m{0}; m < points.size() - 1; m++) {
        for (size_t n{m + 1}; n < points.size(); n++) {
            if (angles.at(m) > angles.at(n)) {
                std::swap(angles[m], angles[n]);
                std::swap(indices[m], indices[n]);
            }
        }
    }

    // reconstruct square in clockwise order
    std::vector<cv::Point2f> output;
    for (const auto &i : indices) {
        output.push_back(cv::Point2f(points[i]));
    }

    return output;
}

std::vector<std::vector<cv::Point2f>> findQuadAreas(const cv::Mat &image,
                                                    int minArea, int maxArea)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, cv::RETR_TREE,
                     cv::CHAIN_APPROX_NONE);

    // for each contour, approximate polygon
    std::vector<std::vector<cv::Point>> contoursPoly;
    std::vector<std::vector<cv::Point2f>> quadrilaterals;
    for (const auto &contour : contours) {
        std::vector<cv::Point> approxCurve;
        cv::approxPolyDP(contour, approxCurve, 5.0, true);
        if (approxCurve.size() == 4 && cv::isContourConvex(approxCurve)) {
            const double area = cv::contourArea(approxCurve);
            if (area > minArea && area < maxArea) {
                quadrilaterals.push_back(sortPoints(approxCurve));
            }
        }
    }

    return quadrilaterals;
}

std::vector<std::vector<cv::Point2f>> findSaddles(
    const cv::Mat &image, std::vector<std::vector<cv::Point2f>> quads)
{
    const cv::TermCriteria criteria{
        cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 400, 0.0001};
    for (const auto &quad : quads) {
        cv::cornerSubPix(image, quad, {9, 9}, {3, 3}, criteria);
    }

    std::vector<size_t> indices;
    for (size_t i{0}; i < 4 * quads.size(); i++) {
        cv::Point2f meanPt{};
        for (size_t n{i}; n < 4 * quads.size(); n++) {
            constexpr double kMaxDistance{20.};
            if (cv::norm(quads[i / 4][i % 4] - quads[n / 4][n % 4]) <
                kMaxDistance) {
                meanPt += quads[n / 4][n % 4];
                indices.push_back(n);
            }
        }

        if (!indices.empty()) {
            meanPt /= static_cast<float>(indices.size());
            while (!indices.empty()) {
                auto n = indices.back();
                indices.pop_back();
                quads[n / 4][n % 4] = meanPt;
            }
        }
    }

    return quads;
}

std::vector<std::vector<cv::Point2f>> findPattern(
    const cv::Mat &inImage, std::vector<std::vector<cv::Point2f>> squares,
    bool flipCalTags)
{
    // CREATE OUTPUT DATA STRUCTURE
    std::vector<std::vector<cv::Point2f>> outputSquares;

    // MAKE LIST OF SQUARE SPACE COORDINATES FOR A SINGLE SQUARE
    std::vector<cv::Point2f> sqPoints;
    sqPoints.emplace_back(0.0f, 0.0f);
    sqPoints.emplace_back(1.0f, 0.0f);
    sqPoints.emplace_back(1.0f, 1.0f);
    sqPoints.emplace_back(0.0f, 1.0f);

    // MAKE LIST OF SQUARE SPACE COORDINATES FOR CODED BITS
    std::vector<cv::Point3f> gdPoints, kgPoints;
    gdPoints.emplace_back(0.6875f, 0.6875f, 1.0f);
    gdPoints.emplace_back(0.6875f, 0.5625f, 1.0f);
    gdPoints.emplace_back(0.6875f, 0.4375f, 1.0f);
    gdPoints.emplace_back(0.6875f, 0.3125f, 1.0f);

    gdPoints.emplace_back(0.5625f, 0.6875f, 1.0f);
    gdPoints.emplace_back(0.5625f, 0.5625f, 1.0f);
    gdPoints.emplace_back(0.5625f, 0.4375f, 1.0f);
    gdPoints.emplace_back(0.5625f, 0.3125f, 1.0f);

    gdPoints.emplace_back(0.4375f, 0.6875f, 1.0f);
    gdPoints.emplace_back(0.4375f, 0.5625f, 1.0f);
    gdPoints.emplace_back(0.4375f, 0.4375f, 1.0f);
    gdPoints.emplace_back(0.4375f, 0.3125f, 1.0f);

    gdPoints.emplace_back(0.3125f, 0.6875f, 1.0f);
    gdPoints.emplace_back(0.3125f, 0.5625f, 1.0f);
    gdPoints.emplace_back(0.3125f, 0.4375f, 1.0f);
    gdPoints.emplace_back(0.3125f, 0.3125f, 1.0f);

    using Matx44 = cv::Matx<int, 4, 4>;

    // CREATE DECODING MATRIX FOR LATER
    Matx44 decodingMatrix;
    for (int i{0}; i < Matx44::channels; i++) {
        decodingMatrix(i / 4, i % 4) = std::round(pow(2.0, (double)i));
    }

    // find the most likely rotation
    int hist[]{0, 0, 0, 0};
    for (const auto &square : squares) {
        // DERIVE SQUARE COORDINATES TO IMAGE PIXEL COORDINATES BASED ON CURRENT
        // SQUARE
        std::vector<cv::Point2f> inPoints;
        for (int c = 0; c < 4; c++) {
            inPoints.push_back(square[c]);
        }
        cv::Mat localTransform =
            cv::getPerspectiveTransform(sqPoints, inPoints);

        // MAP THE CODE BIT COORDINATES TO IMAGE PIXEL COORDINATES
        Matx44 codeMatrix;
        cv::transform(gdPoints, kgPoints, localTransform);
        for (size_t i{0}; i < kgPoints.size(); i++) {
            // ACCUMULATE THE SUM OF PIXELS WITHIN A SMALL WINDOW ABOUT CURRENT
            // PIXEL
            int row = std::round(kgPoints[i].y / kgPoints[i].z);
            int col = std::round(kgPoints[i].x / kgPoints[i].z);

            // MAKE SURE THE CURRENT COORDINATE IS WITHIN THE BOUNDS OF THE
            // IMAGE
            if (row > 0 && row < inImage.rows - 1) {
                if (col > 0 && col < inImage.cols - 1) {
                    int sum = 0;
                    for (int r = row - 1; r < row + 2; r++) {
                        for (int c = col - 1; c < col + 2; c++) {
                            sum += (inImage.at<unsigned char>(r, c) > 128);
                        }
                    }
                    // COPY THE CURRENT BIT TO THE CODE MATRIX
                    codeMatrix(i / 4, 3 - i % 4) = sum > 4 ? 1 : 0;
                }
            }
        }

        // FLIP THE CODE MATRIX LEFT-RIGHT IN CASE WE ARE
        // LOOKING AT THE TARGET FROM BEHIND WITH BACK LIGHTING
        if (flipCalTags) {
            cv::flip(codeMatrix, codeMatrix, 1);
        }

        cv::Point2f point;
        int code = decodingMatrix.dot(codeMatrix);
        if (checkBitCode(code, point)) {
            hist[0]++;
        }

        auto matrix = codeMatrix;
        cv::transpose(matrix, matrix);
        cv::flip(matrix, matrix, 1);
        code = decodingMatrix.dot(matrix);
        if (checkBitCode(code, point)) {
            hist[1]++;
        }

        cv::transpose(matrix, matrix);
        cv::flip(matrix, matrix, 1);
        code = decodingMatrix.dot(matrix);
        if (checkBitCode(code, point)) {
            hist[2]++;
        }

        cv::transpose(matrix, matrix);
        cv::flip(matrix, matrix, 1);
        code = decodingMatrix.dot(matrix);
        if (checkBitCode(code, point)) {
            hist[3]++;
        }
    }

    // DETERMINE WHICH ORIENTATION IS THE MOST FREQUENT
    int orientation = 3;
    if (hist[0] > hist[1] && hist[0] > hist[2] && hist[0] > hist[3]) {
        orientation = 0;
    }
    else if (hist[1] > hist[0] && hist[1] > hist[2] && hist[1] > hist[3]) {
        orientation = 1;
    }
    else if (hist[2] > hist[0] && hist[2] > hist[1] && hist[2] > hist[3]) {
        orientation = 2;
    }

    int validCodeCounter = 0;
    for (const auto &square : squares) {
        // DERIVE SQUARE COORDINATES TO IMAGE PIXEL COORDINATES BASED ON CURRENT
        // SQUARE
        std::vector<cv::Point2f> inPoints;
        for (int c = 0; c < 4; c++) {
            inPoints.push_back(square[c]);
        }

        cv::Mat localTransform =
            cv::getPerspectiveTransform(sqPoints, inPoints);

        // MAP THE CODE BIT COORDINATES TO IMAGE PIXEL COORDINATES
        Matx44 codeMatrix;
        cv::transform(gdPoints, kgPoints, localTransform);
        for (size_t i{0}; i < kgPoints.size(); i++) {
            // ACCUMULATE THE SUM OF PIXELS WITHIN A SMALL WINDOW ABOUT CURRENT
            // PIXEL
            int row = std::round(kgPoints[i].y / kgPoints[i].z);
            int col = std::round(kgPoints[i].x / kgPoints[i].z);

            // MAKE SURE THE CURRENT COORDINATE IS WITHIN THE BOUNDS OF THE
            // IMAGE
            if (row > 0 && row < inImage.rows - 1) {
                if (col > 0 && col < inImage.cols - 1) {
                    int sum = 0;
                    for (int r = row - 1; r < row + 2; r++) {
                        for (int c = col - 1; c < col + 2; c++) {
                            sum += (inImage.at<unsigned char>(r, c) > 128);
                        }
                    }
                    // COPY THE CURRENT BIT TO THE CODE MATRIX
                    codeMatrix(i / 4, 3 - i % 4) = sum > 4 ? 1 : 0;
                }
            }
        }

        // FLIP THE CODE MATRIX LEFT-RIGHT IN CASE WE ARE
        // LOOKING AT THE TARGET FROM BEHIND WITH BACK LIGHTING
        if (flipCalTags) {
            cv::flip(codeMatrix, codeMatrix, 1);

            cv::Point2f point;
            unsigned int code = decodingMatrix.dot(codeMatrix);
            if (orientation != 0 || !checkBitCode(code, point)) {
                auto matrix = codeMatrix;
                cv::transpose(matrix, matrix);
                cv::flip(matrix, matrix, 1);
                code = decodingMatrix.dot(matrix);

                if (orientation != 1 || !checkBitCode(code, point)) {
                    cv::transpose(matrix, matrix);
                    cv::flip(matrix, matrix, 1);
                    code = decodingMatrix.dot(matrix);

                    if (orientation != 2 || !checkBitCode(code, point)) {
                        cv::transpose(matrix, matrix);
                        cv::flip(matrix, matrix, 1);
                        code = decodingMatrix.dot(matrix);

                        if (orientation != 3 || !checkBitCode(code, point)) {
                            std::vector<cv::Point2f> square;
                            square.push_back(cv::Point2f(NAN, NAN));
                            square.push_back(cv::Point2f(NAN, NAN));
                            square.push_back(cv::Point2f(NAN, NAN));
                            square.push_back(cv::Point2f(NAN, NAN));
                            outputSquares.push_back(square);
                        }
                        else {
                            validCodeCounter++;
                            std::vector<cv::Point2f> square;
                            square.push_back(sqPoints[3] + point);
                            square.push_back(sqPoints[2] + point);
                            square.push_back(sqPoints[1] + point);
                            square.push_back(sqPoints[0] + point);
                            outputSquares.push_back(square);
                        }
                    }
                    else {
                        validCodeCounter++;
                        std::vector<cv::Point2f> square;
                        square.push_back(sqPoints[2] + point);
                        square.push_back(sqPoints[1] + point);
                        square.push_back(sqPoints[0] + point);
                        square.push_back(sqPoints[3] + point);
                        outputSquares.push_back(square);
                    }
                }
                else {
                    validCodeCounter++;
                    std::vector<cv::Point2f> square;
                    square.push_back(sqPoints[1] + point);
                    square.push_back(sqPoints[0] + point);
                    square.push_back(sqPoints[3] + point);
                    square.push_back(sqPoints[2] + point);
                    outputSquares.push_back(square);
                }
            }
            else {
                std::vector<cv::Point2f> square;
                square.push_back(sqPoints[0] + point);
                square.push_back(sqPoints[3] + point);
                square.push_back(sqPoints[2] + point);
                square.push_back(sqPoints[1] + point);
                outputSquares.push_back(square);
            }
        }
        else {
            cv::Point2f point;
            int code = decodingMatrix.dot(codeMatrix);
            if (orientation != 0 || !checkBitCode(code, point)) {
                auto matrix = codeMatrix;
                cv::transpose(matrix, matrix);
                cv::flip(matrix, matrix, 1);
                code = decodingMatrix.dot(matrix);

                if (orientation != 1 || !checkBitCode(code, point)) {
                    cv::transpose(matrix, matrix);
                    cv::flip(matrix, matrix, 1);
                    code = decodingMatrix.dot(matrix);

                    if (orientation != 2 || !checkBitCode(code, point)) {
                        cv::transpose(matrix, matrix);
                        cv::flip(matrix, matrix, 1);
                        code = decodingMatrix.dot(matrix);

                        if (orientation != 3 || !checkBitCode(code, point)) {
                            std::vector<cv::Point2f> square;
                            square.push_back(cv::Point2f(NAN, NAN));
                            square.push_back(cv::Point2f(NAN, NAN));
                            square.push_back(cv::Point2f(NAN, NAN));
                            square.push_back(cv::Point2f(NAN, NAN));
                            outputSquares.push_back(square);
                        }
                        else {
                            validCodeCounter++;
                            std::vector<cv::Point2f> square;
                            square.push_back(sqPoints[0] + point);
                            square.push_back(sqPoints[1] + point);
                            square.push_back(sqPoints[2] + point);
                            square.push_back(sqPoints[3] + point);
                            outputSquares.push_back(square);
                        }
                    }
                    else {
                        validCodeCounter++;
                        std::vector<cv::Point2f> square;
                        square.push_back(sqPoints[3] + point);
                        square.push_back(sqPoints[0] + point);
                        square.push_back(sqPoints[1] + point);
                        square.push_back(sqPoints[2] + point);
                        outputSquares.push_back(square);
                    }
                }
                else {
                    validCodeCounter++;
                    std::vector<cv::Point2f> square;
                    square.push_back(sqPoints[2] + point);
                    square.push_back(sqPoints[3] + point);
                    square.push_back(sqPoints[0] + point);
                    square.push_back(sqPoints[1] + point);
                    outputSquares.push_back(square);
                }
            }
            else {
                std::vector<cv::Point2f> square;
                square.push_back(sqPoints[1] + point);
                square.push_back(sqPoints[2] + point);
                square.push_back(sqPoints[3] + point);
                square.push_back(sqPoints[0] + point);
                outputSquares.push_back(square);
            }
        }
    }

    // RETURN THE MATCHING CALTAG SQUARE COORDINATES
    return outputSquares;
}

cv::Mat findBestQuadraticMapping(std::vector<cv::Point2f> fmPoints,
                                 std::vector<cv::Point2f> toPoints, int width,
                                 int height, int order)
{
    int numPoints = (int)fmPoints.size();

    std::vector<double> rVec(numPoints);
    std::vector<double> cVec(numPoints);
    std::vector<double> xVec(numPoints);
    std::vector<double> yVec(numPoints);

    for (int n = 0; n < numPoints; n++) {
        cVec[n] = (fmPoints[n].x - (width / 2)) / 50.0;
        rVec[n] = (fmPoints[n].y - (height / 2)) / 50.0;
        xVec[n] = toPoints[n].x;
        yVec[n] = toPoints[n].y;
    }

    cv::Mat lVec = cv::Mat::zeros(30, 1, CV_64F);
    if (order == 4) {
        if (fmPoints.size() >= 15) {
            cv::Mat A = cv::Mat::zeros(2 * numPoints, 30, CV_64F);
            cv::Mat B{2 * numPoints, 1, CV_64F};

            for (int r = 0; r < numPoints; r++) {
                // POPULATE THE ODD NUMBERED ROWS
                A.at<double>(2 * r + 0, 0) =
                    cVec[r] * cVec[r] * cVec[r] * cVec[r];
                A.at<double>(2 * r + 0, 1) =
                    cVec[r] * cVec[r] * cVec[r] * rVec[r];
                A.at<double>(2 * r + 0, 2) =
                    cVec[r] * cVec[r] * rVec[r] * rVec[r];
                A.at<double>(2 * r + 0, 3) =
                    cVec[r] * rVec[r] * rVec[r] * rVec[r];
                A.at<double>(2 * r + 0, 4) =
                    rVec[r] * rVec[r] * rVec[r] * rVec[r];
                A.at<double>(2 * r + 0, 5) = cVec[r] * cVec[r] * cVec[r];
                A.at<double>(2 * r + 0, 6) = cVec[r] * cVec[r] * rVec[r];
                A.at<double>(2 * r + 0, 7) = cVec[r] * rVec[r] * rVec[r];
                A.at<double>(2 * r + 0, 8) = rVec[r] * rVec[r] * rVec[r];
                A.at<double>(2 * r + 0, 9) = cVec[r] * cVec[r];
                A.at<double>(2 * r + 0, 10) = cVec[r] * rVec[r];
                A.at<double>(2 * r + 0, 11) = rVec[r] * rVec[r];
                A.at<double>(2 * r + 0, 12) = cVec[r];
                A.at<double>(2 * r + 0, 13) = rVec[r];
                A.at<double>(2 * r + 0, 14) = 1.0;

                A.at<double>(2 * r + 1, 15) =
                    cVec[r] * cVec[r] * cVec[r] * cVec[r];
                A.at<double>(2 * r + 1, 16) =
                    cVec[r] * cVec[r] * cVec[r] * rVec[r];
                A.at<double>(2 * r + 1, 17) =
                    cVec[r] * cVec[r] * rVec[r] * rVec[r];
                A.at<double>(2 * r + 1, 18) =
                    cVec[r] * rVec[r] * rVec[r] * rVec[r];
                A.at<double>(2 * r + 1, 19) =
                    rVec[r] * rVec[r] * rVec[r] * rVec[r];
                A.at<double>(2 * r + 1, 20) = cVec[r] * cVec[r] * cVec[r];
                A.at<double>(2 * r + 1, 21) = cVec[r] * cVec[r] * rVec[r];
                A.at<double>(2 * r + 1, 22) = cVec[r] * rVec[r] * rVec[r];
                A.at<double>(2 * r + 1, 23) = rVec[r] * rVec[r] * rVec[r];
                A.at<double>(2 * r + 1, 24) = cVec[r] * cVec[r];
                A.at<double>(2 * r + 1, 25) = cVec[r] * rVec[r];
                A.at<double>(2 * r + 1, 26) = rVec[r] * rVec[r];
                A.at<double>(2 * r + 1, 27) = cVec[r];
                A.at<double>(2 * r + 1, 28) = rVec[r];
                A.at<double>(2 * r + 1, 29) = 1.0;

                B.at<double>(2 * r + 0) = xVec[r];
                B.at<double>(2 * r + 1) = yVec[r];
            }
            lVec = (A.t() * A).inv() * A.t() * B;
        }
    }
    else if (order == 3) {
        if (fmPoints.size() >= 10) {
            cv::Mat A(2 * numPoints, 20, CV_64F);
            A.setTo(0.0);
            cv::Mat B(2 * numPoints, 1, CV_64F);
            for (int r = 0; r < numPoints; r++) {
                // POPULATE THE ODD NUMBERED ROWS
                A.at<double>(2 * r + 0, 0) = cVec[r] * cVec[r] * cVec[r];
                A.at<double>(2 * r + 0, 1) = cVec[r] * cVec[r] * rVec[r];
                A.at<double>(2 * r + 0, 2) = cVec[r] * rVec[r] * rVec[r];
                A.at<double>(2 * r + 0, 3) = rVec[r] * rVec[r] * rVec[r];
                A.at<double>(2 * r + 0, 4) = cVec[r] * cVec[r];
                A.at<double>(2 * r + 0, 5) = cVec[r] * rVec[r];
                A.at<double>(2 * r + 0, 6) = rVec[r] * rVec[r];
                A.at<double>(2 * r + 0, 7) = cVec[r];
                A.at<double>(2 * r + 0, 8) = rVec[r];
                A.at<double>(2 * r + 0, 9) = 1.0;

                A.at<double>(2 * r + 1, 10) = cVec[r] * cVec[r] * cVec[r];
                A.at<double>(2 * r + 1, 11) = cVec[r] * cVec[r] * rVec[r];
                A.at<double>(2 * r + 1, 12) = cVec[r] * rVec[r] * rVec[r];
                A.at<double>(2 * r + 1, 13) = rVec[r] * rVec[r] * rVec[r];
                A.at<double>(2 * r + 1, 14) = cVec[r] * cVec[r];
                A.at<double>(2 * r + 1, 15) = cVec[r] * rVec[r];
                A.at<double>(2 * r + 1, 16) = rVec[r] * rVec[r];
                A.at<double>(2 * r + 1, 17) = cVec[r];
                A.at<double>(2 * r + 1, 18) = rVec[r];
                A.at<double>(2 * r + 1, 19) = 1.0;

                B.at<double>(2 * r + 0) = xVec[r];
                B.at<double>(2 * r + 1) = yVec[r];
            }
            cv::Mat sVec = (A.t() * A).inv() * A.t() * B;

            // COPY SVECTOR OVER TO LVECTOR
            for (int n = 0; n < 10; n++) {
                lVec.at<double>(n + 5) = sVec.at<double>(n + 0);
                lVec.at<double>(n + 20) = sVec.at<double>(n + 10);
            }
        }
    }
    else if (order == 2) {
        if (fmPoints.size() >= 6) {
            cv::Mat A(2 * numPoints, 12, CV_64F);
            A.setTo(0.0);
            cv::Mat B(2 * numPoints, 1, CV_64F);

            for (int r = 0; r < numPoints; r++) {
                // POPULATE THE ODD NUMBERED ROWS
                A.at<double>(2 * r + 0, 0) = cVec[r] * cVec[r];
                A.at<double>(2 * r + 0, 1) = cVec[r] * rVec[r];
                A.at<double>(2 * r + 0, 2) = rVec[r] * rVec[r];
                A.at<double>(2 * r + 0, 3) = cVec[r];
                A.at<double>(2 * r + 0, 4) = rVec[r];
                A.at<double>(2 * r + 0, 5) = 1.0;

                A.at<double>(2 * r + 1, 6) = cVec[r] * cVec[r];
                A.at<double>(2 * r + 1, 7) = cVec[r] * rVec[r];
                A.at<double>(2 * r + 1, 8) = rVec[r] * rVec[r];
                A.at<double>(2 * r + 1, 9) = cVec[r];
                A.at<double>(2 * r + 1, 10) = rVec[r];
                A.at<double>(2 * r + 1, 11) = 1.0;

                B.at<double>(2 * r + 0) = xVec[r];
                B.at<double>(2 * r + 1) = yVec[r];
            }
            cv::Mat sVec = (A.t() * A).inv() * A.t() * B;

            for (int n = 0; n < 6; n++) {
                lVec.at<double>(n + 9) = sVec.at<double>(n + 0);
                lVec.at<double>(n + 24) = sVec.at<double>(n + 6);
            }
        }
    }
    else if (order == 1) {
        if (fmPoints.size() >= 3) {
            cv::Mat A(2 * numPoints, 6, CV_64F);
            A.setTo(0.0);
            cv::Mat B(2 * numPoints, 1, CV_64F);

            for (int r = 0; r < numPoints; r++) {
                // POPULATE THE ODD NUMBERED ROWS
                A.at<double>(2 * r + 0, 0) = cVec[r];
                A.at<double>(2 * r + 0, 1) = rVec[r];
                A.at<double>(2 * r + 0, 2) = 1.0;

                A.at<double>(2 * r + 1, 3) = cVec[r];
                A.at<double>(2 * r + 1, 4) = rVec[r];
                A.at<double>(2 * r + 1, 5) = 1.0;

                B.at<double>(2 * r + 0) = xVec[r];
                B.at<double>(2 * r + 1) = yVec[r];
            }
            cv::Mat sVec = (A.t() * A).inv() * A.t() * B;

            for (int n = 0; n < 3; n++) {
                lVec.at<double>(n + 12) = sVec.at<double>(n + 0);
                lVec.at<double>(n + 27) = sVec.at<double>(n + 3);
            }
        }
    }

    return lVec;
}

void removeOutlierPoints(std::vector<cv::Point2f> &srcPoints,
                         std::vector<cv::Point2f> &dstPoints)
{
    constexpr size_t kMinPoint{20};
    if (srcPoints.size() < kMinPoint) {
        return;
    }

    const size_t numIterations = srcPoints.size() / 2;
    for (size_t iter{0}; iter < numIterations; iter++) {
        cv::Mat transform = cv::findHomography(srcPoints, dstPoints);

        std::vector<cv::Point2f> reprojPoints;
        cv::perspectiveTransform(srcPoints, reprojPoints, transform);

        double max_err{0.0};
        size_t max_idx{0};
        for (size_t n{0}; n < srcPoints.size(); n++) {
            double err = cv::norm(dstPoints[n] - reprojPoints[n]);
            if (err > max_err) {
                max_idx = n;
                max_err = err;
            }
        }

        constexpr double kReprojErrThres{0.05};
        if (max_err < kReprojErrThres) {
            break;
        }

        srcPoints.erase(srcPoints.begin() + max_idx);
        dstPoints.erase(dstPoints.begin() + max_idx);
    }
}

struct Pairing
{
    cv::Point cr;
    cv::Point xy;
};

bool detectCalTagGrid(cv::Mat rwImage, cv::Mat sbImage, cv::Mat inImage,
                      int minBoxes, double minRegion, double maxRegion,
                      bool flipCalTags, std::vector<Pairing> &pairings,
                      cv::Mat &transform)

{
    pairings.clear();

    // 1. find quads
    std::vector<std::vector<cv::Point2f>> quads =
        findQuadAreas(sbImage, minRegion, maxRegion);

    // 2. refine quads
    quads = findSaddles(rwImage, quads);
    // quads = organizeSquares(quads);

    // 3. decode
    std::vector<std::vector<cv::Point2f>> coordinates =
        findPattern(inImage, quads, flipCalTags);

    if (quads.size() <= (unsigned long)minBoxes) {
        return false;
    }

    // DELETE INVALID SQUARES WHILE ACCUMULATING POINTS FROM IMAGE TO GRID
    // SPACE
    std::vector<cv::Point2f> toPoints, fmPoints;
    for (size_t i{0}; i < coordinates.size(); i++) {
        for (int j{0}; j < 4; j++) {
            if (!math::isApprox0(coordinates[i][j].x * coordinates[i][j].y)) {
                fmPoints.push_back(quads[i][j]);
                toPoints.push_back(coordinates[i][j]);
            }
        }
    }

    // REMOVE OUTLIERS FROM OUR FMPOINTS BY USING A LINEAR MAPPING
    // BETWEEN THE TWO
    removeOutlierPoints(fmPoints, toPoints);

    // MAKE SURE WE HAVE ENOUGH DETECTED POINTS
    if (toPoints.size() <= (unsigned long)minBoxes) {
        return false;
    }

    // MAKE A LOCAL COPY OF THE POINT PAIRINGS
    for (size_t n{0}; n < toPoints.size() && n < fmPoints.size(); n++) {
        Pairing pairing;
        pairing.cr = cv::Point(fmPoints[n]);
        pairing.xy = cv::Point(toPoints[n]);
        pairings.push_back(pairing);
    }

    // find best transform from camera to board
    constexpr int kPolynomialOrder{2};
    transform = findBestQuadraticMapping(fmPoints, toPoints, inImage.cols,
                                         inImage.rows, kPolynomialOrder);

    return true;
}

} // namespace tl
