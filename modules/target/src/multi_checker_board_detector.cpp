#include "multi_checker_board_detector.h"

#include <numeric>
#include <opencv2/imgproc.hpp>

namespace tl {

namespace {
inline float normpdf(float x, float mu, float sigma)
{
    return std::exp(-0.5f * (x - mu) * (x - mu) / (sigma * sigma)) /
           (std::sqrt((float)CV_2PI) * sigma);
}

template <typename T>
inline T mean(const std::vector<T> &values)
{
    return std::accumulate(values.begin(), values.end(), T(0)) / values.size();
}

template <typename T>
inline T stddev(const std::vector<T> &values, T &avg)
{
    avg = mean(values);
    T dev = std::accumulate(values.cbegin(), values.cend(), T(0),
                            [&avg](const T &tmp, const T &val) {
                                return tmp + (val - avg) * (val - avg);
                            });

    return std::sqrt(dev / (values.size() - 1));
}

template <typename T>
inline T stddevMean(const std::vector<T> &values)
{
    T mean;
    T stdd = stddev(values, mean);
    return stdd / mean;
}

inline double distance(const cv::Vec2f &a, const cv::Vec2f &b = {})
{
    return cv::norm(a - b);
}

void calcImageAngleAndWeight(cv::Mat img, cv::Mat &img_du, cv::Mat &img_dv,
                             cv::Mat &img_angle, cv::Mat &img_weight)
{
    // sobel kernel, TODO: why not cv::sobel()
    cv::Mat hKernel(3, 3, CV_32F);
    hKernel.col(0).setTo(cv::Scalar(-1.f));
    hKernel.col(1).setTo(cv::Scalar(0.f));
    hKernel.col(2).setTo(cv::Scalar(1.f));
    cv::Mat vKernel = hKernel.t();

    cv::filter2D(img, img_du, CV_32F, hKernel);
    cv::filter2D(img, img_dv, CV_32F, vKernel);

    cv::cartToPolar(img_du, img_dv, img_weight, img_angle);

    // clamp atan
    img_angle.forEach<float>([](float &val, const int * /*pos*/) {
        if (val < 0.f)
            val += (float)CV_PI;
        else if (val > (float)CV_PI)
            val -= (float)CV_PI;
    });
}

} // namespace

/// ------- MultiCheckerBoardDetector impl starts from here
class MultiCheckerBoardDetector::Impl
{
public:
    Impl();

    void init(const Options &opts);

    // corners
    void createKernels(float angle1, float angle2, int kernelSize,
                       cv::Mat &kernelA, cv::Mat &kernelB, cv::Mat &kernelC,
                       cv::Mat &kernelD) const;

    void nonMaximumSuppression(const cv::Mat &imageCorners,
                               std::vector<cv::Point2f> &corners, int patchSize,
                               int margin, float threshold) const;

    void refineCorners(std::vector<cv::Point2f> &corners, const cv::Mat &du,
                       const cv::Mat &dv, const cv::Mat &imageAngle,
                       const cv::Mat &imageWeight, float radius);

    void scoreCorners(const cv::Mat &image, const cv::Mat &imageWeight,
                      const std::vector<cv::Point2f> &corners,
                      const std::vector<int> &radii,
                      std::vector<float> &score) const;

    void resizeCornersEdge(int size);

    // chessboard
    cv::Mat initChessboard(const Corners &corners, int index);

    float chessboardEnergy(const cv::Mat &chessboard,
                           const Corners &corners) const;

    cv::Mat growChessboard(const cv::Mat &chessboard, const Corners &corners,
                           int borderType) const;

public:
    std::vector<cv::Vec2f> m_cornersEdge1;
    std::vector<cv::Vec2f> m_cornersEdge2;
    cv::Mat m_chessboard; // TODO: maybe remove this
    Options m_opts;

private:
    void updateCornerEdges(const cv::Mat &imageAngle,
                           const cv::Mat &imageWeight, size_t index);

    void findModesMeanShift(const std::vector<float> &histogram,
                            std::vector<float> &smoothedHistorgram,
                            std::vector<std::pair<float, int>> &modes,
                            float sigma);

    float scoreCorner(const cv::Mat &image, const cv::Mat &imageWeight,
                      const std::array<cv::Vec2f, 2> &edgePair) const;

    // chessboard
    int directionalNeighbor(int index, const cv::Vec2f &edge,
                            const cv::Mat &chessboard, const Corners &corners,
                            int &neighborIndex, float &minDistance) const;

    void predictCorners(const std::vector<cv::Vec2f> &p1,
                        const std::vector<cv::Vec2f> &p2,
                        const std::vector<cv::Vec2f> &p3,
                        std::vector<cv::Vec2f> &predictions) const;

    void assignClosestCorners(const std::vector<cv::Vec2f> &candidates,
                              const std::vector<cv::Vec2f> &preditions,
                              std::vector<int> &indexes) const;

private:
};

MultiCheckerBoardDetector::Impl::Impl() {}

void MultiCheckerBoardDetector::Impl::init(const Options &opts)
{
    m_opts = opts;
}

void MultiCheckerBoardDetector::Impl::createKernels(
    float angle1, float angle2, int kernelSize, cv::Mat &kerA, cv::Mat &kerB,
    cv::Mat &kerC, cv::Mat &kerD) const
{
    const int cols = kernelSize * 2 + 1;
    const int rows = kernelSize * 2 + 1;
    kerA = cv::Mat::zeros(rows, cols, CV_32F);
    kerB = cv::Mat::zeros(rows, cols, CV_32F);
    kerC = cv::Mat::zeros(rows, cols, CV_32F);
    kerD = cv::Mat::zeros(rows, cols, CV_32F);

    for (int c{0}; c < cols; ++c) {
        for (int r{0}; r < rows; ++r) {
            auto offset = cv::Vec2f(cv::Vec2i{c - kernelSize, r - kernelSize});
            double dist = cv::norm(offset);
            float val = normpdf(dist, 0.f, kernelSize / 2.f);
            // 2D rotate at kernel center
            // X = X0 * cos + Y0 * sin
            // Y = Y0 * cos - X0 * sin
            float side1 = offset[0] * (-sin(angle1)) + offset[1] * cos(angle1);
            float side2 = offset[0] * (-sin(angle2)) + offset[1] * cos(angle2);

            if (side1 <= -0.1 && side2 <= -0.1) {
                kerA.ptr<float>(r)[c] = val;
            }
            if (side1 >= 0.1 && side2 >= 0.1) {
                kerB.ptr<float>(r)[c] = val;
            }
            if (side1 <= -0.1 && side2 >= 0.1) {
                kerC.ptr<float>(r)[c] = val;
            }
            if (side1 >= 0.1 && side2 <= -0.1) {
                kerD.ptr<float>(r)[c] = val;
            }
        }
    }

    kerA = kerA / cv::sum(kerA)[0];
    kerB = kerB / cv::sum(kerB)[0];
    kerC = kerC / cv::sum(kerC)[0];
    kerD = kerD / cv::sum(kerD)[0];
}

void MultiCheckerBoardDetector::Impl::nonMaximumSuppression(
    const cv::Mat &img_corners, std::vector<cv::Point2f> &corners,
    int patchSize, int margin, float threshold) const
{
    CV_Assert(!img_corners.empty());

    const int cols = img_corners.cols;
    const int rows = img_corners.rows;
    for (int x{margin + patchSize}; x < cols - (margin + patchSize);
         x += (patchSize + 1)) {
        for (int y{margin + patchSize}; y < rows - (margin + patchSize);
             y += (patchSize + 1)) {
            auto maxVal = img_corners.ptr<float>(y)[x];
            int maxX = x;
            int maxY = y;
            for (int u{x}; u <= x + patchSize; u++) {
                for (int v{y}; v <= y + patchSize; v++) {
                    auto temp = img_corners.ptr<float>(v)[u];
                    if (temp > maxVal) {
                        maxVal = temp;
                        maxX = u;
                        maxY = v;
                    }
                }
            }

            if (maxVal < threshold)
                continue;

            // verify in neighbor area
            bool notMax{false};
            const int uMax = std::min(maxX + patchSize, cols - margin - 1);
            const int vMax = std::min(maxY + patchSize, rows - margin - 1);
            const int uMin = maxX - patchSize;
            const int vMin = maxY - patchSize;
            for (int u{uMin}; u <= uMax; u++) {
                for (int v{vMin}; v <= vMax; v++) {
                    if (img_corners.ptr<float>(v)[u] > maxVal &&
                        (u < x || u > x + patchSize || v < y ||
                         v > y + patchSize)) {
                        notMax = true;
                        break;
                    }
                }

                if (notMax)
                    break;
            }

            if (notMax)
                continue;

            // FIXME: int -> float
            corners.push_back(cv::Point{maxX, maxY});
        }
    }
}

void MultiCheckerBoardDetector::Impl::refineCorners(
    std::vector<cv::Point2f> &corners, const cv::Mat &img_du,
    const cv::Mat &img_dv, const cv::Mat &img_angle, const cv::Mat &img_weight,
    float radius)
{
    for (size_t i{0}; i < corners.size(); i++) {
        auto cu = corners[i].x;
        auto cv = corners[i].y;

        // estimate edge orientations
        auto roi_x = std::max(cu - radius, 0.f);
        auto roi_y = std::max(cv - radius, 0.f);
        auto roi_w = std::min(cu + radius + 1.f, img_du.cols - 1.f) - roi_x;
        auto roi_h = std::min(cv + radius + 1.f, img_du.rows - 1.f) - roi_y;
        cv::Rect2f roi_raw{roi_x, roi_y, roi_w, roi_h};
        cv::Rect roi(roi_raw);

        cv::Mat roi_angle = img_angle(roi);
        cv::Mat roi_weight = img_weight(roi);
        updateCornerEdges(roi_angle, roi_weight, i);

        if ((m_cornersEdge1[i] == cv::Vec2f{}) ||
            (m_cornersEdge2[i] == cv::Vec2f{})) {
            continue;
        }

        constexpr float kOrientThres{0.25f};

        // corner orientation refinement
        cv::Matx22f A1 = cv::Matx22f::zeros();
        cv::Matx22f A2 = cv::Matx22f::zeros();
        for (int x{roi.x}; x < roi.x + roi.width; x++) {
            for (int y{roi.y}; y < roi.y + roi.height; y++) {
                auto du = img_du.at<float>(y, x);
                auto dv = img_dv.at<float>(y, x);

                cv::Vec2f orient{du, dv};
                auto unit_orient = cv::normalize(orient);
                auto addtionu = du * orient;
                auto addtionv = dv * orient;

                float t0 = std::abs(unit_orient.dot(m_cornersEdge1[i]));
                if (t0 < kOrientThres) {
                    for (int j{0}; j < A1.cols; j++) {
                        A1(0, j) += addtionu(j);
                        A1(1, j) += addtionv(j);
                    }
                }

                float t1 = std::abs(unit_orient.dot(m_cornersEdge2[i]));
                if (t1 < kOrientThres) {
                    for (int j{0}; j < A2.cols; j++) {
                        A2(0, j) += addtionu(j);
                        A2(1, j) += addtionv(j);
                    }
                }
            }
        }

        // set new corner orientation
        cv::Mat eigenvalues1, eigenvectors1;
        cv::Mat eigenvalues2, eigenvectors2;
        cv::eigen(A1, eigenvalues1, eigenvectors1);
        cv::eigen(A2, eigenvalues2, eigenvectors2);
        m_cornersEdge1[i][0] = -eigenvectors1.at<float>(1, 0);
        m_cornersEdge1[i][1] = -eigenvectors1.at<float>(1, 1);
        m_cornersEdge2[i][0] = -eigenvectors2.at<float>(1, 0);
        m_cornersEdge2[i][1] = -eigenvectors2.at<float>(1, 1);

        //  corner location refinement
        auto G = cv::Matx22f::zeros();
        auto b = cv::Vec2f::all(0.f);
        for (int x{roi.x}; x < roi.x + roi.width; x++) {
            for (int y{roi.y}; y < roi.y + roi.height; y++) {
                auto du = img_du.at<float>(y, x);
                auto dv = img_dv.at<float>(y, x);

                cv::Vec2f orient{du, dv};
                auto unit_orient = cv::normalize(orient);

                // do not consider center pixel
                if (x != cu || y != cv) {
                    // compute rel. position of pixel and distance to vectors
                    cv::Vec2f w(x - cu, y - cv);
                    float wvv1 = w.dot(m_cornersEdge1[i]);
                    float wvv2 = w.dot(m_cornersEdge2[i]);
                    cv::Vec2f wv1 = wvv1 * m_cornersEdge1[i];
                    cv::Vec2f wv2 = wvv2 * m_cornersEdge2[i];
                    cv::Vec2f vd1 = w - wv1;
                    cv::Vec2f vd2 = w - wv2;
                    double d1 = cv::norm(vd1);
                    double d2 = cv::norm(vd2);
                    // if pixel corresponds with either of the vectors /
                    // directions
                    if (((d1 < 3) && std::abs(unit_orient.dot(
                                         m_cornersEdge1[i])) < kOrientThres) ||
                        ((d2 < 3) && std::abs(unit_orient.dot(
                                         m_cornersEdge2[i])) < kOrientThres)) {
                        cv::Vec2f uvt = cv::Vec2f(cv::Vec2i{x, y});
                        cv::Matx22f H{du * du, du * dv, dv * du, dv * dv};
                        G += H;
                        b += (H * uvt);
                    }
                }
            }
        }

        // set new corner location if G has full rank
        cv::Mat s, u, v;
        cv::SVD::compute(G, s, u, v);
        int rank = 0;
        for (int k = 0; k < s.rows; k++) {
            // not equal zero
            if (s.at<float>(k, 0) > 0.0001 || s.at<float>(k, 0) < -0.0001) {
                rank++;
            }
        }

        if (rank == 2) {
            auto corner_pos_new = G.inv() * b;
            //  % set corner to invalid, if position update is very large
            if (cv::norm(corner_pos_new - cv::Vec2f(corners[i])) >= 4) {
                m_cornersEdge1[i] = {};
                m_cornersEdge2[i] = {};
            }
            else {
                corners[i] = {corner_pos_new};
            }
        }
        else // otherwise: set corner to invalid
        {
            m_cornersEdge1[i] = {};
            m_cornersEdge2[i] = {};
        }
    }
}

void MultiCheckerBoardDetector::Impl::scoreCorners(
    const cv::Mat &img, const cv::Mat &img_weight,
    const std::vector<cv::Point2f> &corners, const std::vector<int> &radii,
    std::vector<float> &scores) const
{
    scores.clear();
    scores.reserve(corners.size());
    for (size_t i{0}; i < corners.size(); i++) {
        // TODO: check precision
        auto corner = cv::Point(corners[i]);

        std::vector<float> statistics;
        statistics.reserve(radii.size());
        for (const auto &r : radii) {
            // TODO: check precision
            if (cv::Rect{r, r, img.cols - 2 * r, img.rows - 2 * r}.contains(
                    corner)) {
                cv::Rect roi{corner - cv::Point{r, r},
                             cv::Size{2 * r + 1, 2 * r + 1}};
                cv::Mat roi_img = img(roi).clone();
                cv::Mat roi_img_weight = img_weight(roi).clone();
                std::array<cv::Vec2f, 2> edge_pair{m_cornersEdge1[i],
                                                   m_cornersEdge2[i]};
                statistics.push_back(
                    scoreCorner(roi_img, roi_img_weight, edge_pair));
            }
        }

        scores.push_back(
            *std::max_element(statistics.begin(), statistics.end()));
    }
}

cv::Mat MultiCheckerBoardDetector::Impl::initChessboard(const Corners &corners,
                                                        int idx)
{
    constexpr int kMinCornerCount{9};
    if (corners.pt.size() < kMinCornerCount) {
        m_chessboard.release();
        return m_chessboard;
    }

    // init chessboard hypothesis
    m_chessboard = -cv::Mat::ones(3, 3, CV_32S);

    // extract feature index and orientation(central element)
    auto &v1 = corners.v1[idx];
    auto &v2 = corners.v2[idx];
    m_chessboard.at<int>(1, 1) = idx;
    std::vector<float> dist1(2), dist2(6);

    // find left / right / top / bottom neighbors
    directionalNeighbor(idx, v1, m_chessboard, corners,
                        m_chessboard.at<int>(1, 2), dist1[0]);
    directionalNeighbor(idx, -v1, m_chessboard, corners,
                        m_chessboard.at<int>(1, 0), dist1[1]);
    directionalNeighbor(idx, v2, m_chessboard, corners,
                        m_chessboard.at<int>(2, 1), dist2[0]);
    directionalNeighbor(idx, -v2, m_chessboard, corners,
                        m_chessboard.at<int>(0, 1), dist2[1]);

    // find top-left / top-right / bottom-left / bottom-right neighbors
    directionalNeighbor(m_chessboard.at<int>(1, 0), -v2, m_chessboard, corners,
                        m_chessboard.at<int>(0, 0), dist2[2]);
    directionalNeighbor(m_chessboard.at<int>(1, 0), v2, m_chessboard, corners,
                        m_chessboard.at<int>(2, 0), dist2[3]);
    directionalNeighbor(m_chessboard.at<int>(1, 2), -v2, m_chessboard, corners,
                        m_chessboard.at<int>(0, 2), dist2[4]);
    directionalNeighbor(m_chessboard.at<int>(1, 2), v2, m_chessboard, corners,
                        m_chessboard.at<int>(2, 2), dist2[5]);

    // initialization must be homogenously distributed
    bool sigood{false};
    sigood = sigood || (dist1[0] < 0) || (dist1[1] < 0);
    sigood = sigood || (dist2[0] < 0) || (dist2[1] < 0) || (dist2[2] < 0) ||
             (dist2[3] < 0) || (dist2[4] < 0) || (dist2[5] < 0);

    sigood = sigood || (stddevMean(dist1) > 0.3) || (stddevMean(dist2) > 0.3);

    if (sigood) {
        m_chessboard.release();
        return m_chessboard;
    }

    return m_chessboard;
}

float MultiCheckerBoardDetector::Impl::chessboardEnergy(
    const cv::Mat &chessboard, const Corners &corners) const
{
    float E_corners = -static_cast<float>(chessboard.size().area());

    double E_structure{0.};
    // by row
    for (int r{0}; r < chessboard.rows; r++) {
        for (int c{0}; c < chessboard.cols - 2; c++) {
            std::vector<cv::Vec2f> triple;
            for (int i{c}; i <= c + 2; i++) {
                int n = chessboard.at<int>(r, i);
                triple.push_back(corners.pt[n]);
            }

            double energy = distance(triple[0] + triple[2] - 2 * triple[1]) /
                            distance(triple[0] - triple[2]);
            E_structure = std::max(E_structure, energy);
        }
    }

    // by col
    for (int c{0}; c < chessboard.cols; c++) {
        for (int r{0}; r < chessboard.rows - 2; r++) {
            std::vector<cv::Vec2f> triple;
            for (int i{r}; i <= r + 2; i++) {
                int n = chessboard.at<int>(i, c);
                triple.push_back(corners.pt[n]);
            }

            double energy = distance(triple[0] + triple[2] - 2 * triple[1]) /
                            distance(triple[0] - triple[2]);
            E_structure = std::max(E_structure, energy);
        }
    }

    return static_cast<float>(
        E_corners + m_opts.lamda * chessboard.size().area() * E_structure);
}

cv::Mat MultiCheckerBoardDetector::Impl::growChessboard(
    const cv::Mat &chessboard, const Corners &corners, int border_type) const
{
    if (chessboard.empty()) {
        return {};
    }

    std::vector<cv::Point2f> p = corners.pt;
    // list of  unused feature elements
    std::vector<int> unused(p.size());
    std::iota(unused.begin(), unused.end(), 0);

    chessboard.forEach<int>([&unused](int &val, const int *pos) {
        if (val >= 0) {
            unused[val] = -1;
        }
    });

    int nsize = unused.size();
    for (int i = 0; i < nsize;) {
        if (unused[i] < 0) {
            auto iter = unused.begin() + i;
            unused.erase(iter);
            i = 0;
            nsize = unused.size();
            continue;
        }
        i++;
    }

    // candidates from unused corners
    std::vector<cv::Vec2f> cand;
    for (int i = 0; i < unused.size(); i++) {
        cand.push_back(corners.pt[unused[i]]);
    }

    cv::Mat ret;
    switch (border_type) {
        case 0: {
            std::vector<cv::Vec2f> p1, p2, p3;
            for (int r{0}; r < chessboard.rows; r++) {
                for (int c{0}; c < chessboard.cols; c++) {
                    int ij = chessboard.at<int>(r, c);
                    if (c == chessboard.cols - 3) {
                        p1.push_back(cv::Vec2f(p[ij]));
                    }
                    if (c == chessboard.cols - 2) {
                        p2.push_back(cv::Vec2f(p[ij]));
                    }
                    if (c == chessboard.cols - 1) {
                        p3.push_back(cv::Vec2f(p[ij]));
                    }
                }
            }

            std::vector<cv::Vec2f> preds;
            predictCorners(p1, p2, p3, preds);

            std::vector<int> indexes;
            assignClosestCorners(cand, preds, indexes);
            if (indexes[0] < 0) {
                return chessboard;
            }

            cv::Mat chess;
            cv::copyMakeBorder(chessboard, chess, 0, 0, 0, 1, 0, 0);

            for (int r{0}; r < chess.rows; r++) {
                chess.at<int>(r, chess.cols - 1) = unused[indexes[r]];
            }

            chess.copyTo(ret);

            break;
        }
        case 1: {
            std::vector<cv::Vec2f> p1, p2, p3;
            for (int r{0}; r < chessboard.rows; r++) {
                for (int c{0}; c < chessboard.cols; c++) {
                    int ij = chessboard.at<int>(r, c);
                    if (r == chessboard.rows - 3) {
                        p1.push_back(cv::Vec2f(p[ij]));
                    }
                    if (r == chessboard.rows - 2) {
                        p2.push_back(cv::Vec2f(p[ij]));
                    }
                    if (r == chessboard.rows - 1) {
                        p3.push_back(cv::Vec2f(p[ij]));
                    }
                }
            }

            std::vector<cv::Vec2f> preds;
            predictCorners(p1, p2, p3, preds);

            std::vector<int> indexes;
            assignClosestCorners(cand, preds, indexes);
            if (indexes[0] < 0) {
                return chessboard;
            }

            cv::Mat chess;
            cv::copyMakeBorder(chessboard, chess, 0, 1, 0, 0, 0, 0);

            for (int c{0}; c < chess.cols; c++) {
                chess.at<int>(chess.rows - 1, c) = unused[indexes[c]];
            }
            chess.copyTo(ret);

            break;
        }
        case 2: {
            std::vector<cv::Vec2f> p1, p2, p3;
            for (int r{0}; r < chessboard.rows; r++) {
                for (int c{0}; c < chessboard.cols; c++) {
                    int ij = chessboard.at<int>(r, c);
                    if (c == 2) {
                        p1.push_back(cv::Vec2f(p[ij]));
                    }
                    if (c == 1) {
                        p2.push_back(cv::Vec2f(p[ij]));
                    }
                    if (c == 0) {
                        p3.push_back(cv::Vec2f(p[ij]));
                    }
                }
            }

            std::vector<cv::Vec2f> preds;
            predictCorners(p1, p2, p3, preds);

            std::vector<int> indexes;
            assignClosestCorners(cand, preds, indexes);
            if (indexes[0] < 0) {
                return chessboard;
            }

            cv::Mat chess;
            cv::copyMakeBorder(chessboard, chess, 0, 0, 1, 0, 0, 0);
            for (int i{0}; i < chess.rows; i++) {
                chess.at<int>(i, 0) = unused[indexes[i]];
            }

            chess.copyTo(ret);

            break;
        }
        case 3: {
            std::vector<cv::Vec2f> p1, p2, p3;
            for (int r{0}; r < chessboard.rows; r++) {
                for (int c{0}; c < chessboard.cols; c++) {
                    int ij = chessboard.at<int>(r, c);
                    if (r == 2) {
                        p1.push_back(cv::Vec2f(p[ij]));
                    }
                    if (r == 1) {
                        p2.push_back(cv::Vec2f(p[ij]));
                    }
                    if (r == 0) {
                        p3.push_back(cv::Vec2f(p[ij]));
                    }
                }
            }

            std::vector<cv::Vec2f> preds;
            predictCorners(p1, p2, p3, preds);

            std::vector<int> indexes;
            assignClosestCorners(cand, preds, indexes);
            if (indexes[0] < 0) {
                return chessboard;
            }

            cv::Mat chess;
            cv::copyMakeBorder(chessboard, chess, 1, 0, 0, 0, 0, 0);
            for (int c{0}; c < chess.cols; c++) {
                chess.at<int>(0, c) = unused[indexes[c]];
            }

            chess.copyTo(ret);

            break;
        }
        default:
            break;
    }

    return {};
}

void MultiCheckerBoardDetector::Impl::resizeCornersEdge(int size)
{
    m_cornersEdge1.resize(size, {});
    m_cornersEdge2.resize(size, {});
}

void MultiCheckerBoardDetector::Impl::updateCornerEdges(
    const cv::Mat &img_angle, const cv::Mat &img_weight, size_t index)
{
    // TODO: check dtype
    CV_Assert(img_angle.size() == img_weight.size() && img_angle.empty());

    // Mat to vector
    std::vector<float> angles;
    img_angle.forEach<float>([&angles](float &val, const int * /*pos*/) {
        // convert angles from normals to directions
        float angle = val + (float)CV_PI / 2.f;
        angle = angle > (float)CV_PI ? (angle - (float)CV_PI) : angle;
        angles.push_back(angle);
    });

    std::vector<float> weights;
    img_weight.forEach<float>([&weights](float &val, const int * /*pos*/) {
        weights.push_back(val);
    });

    // create histogram
    constexpr int binCount{32};
    constexpr float binRange{(float)CV_PI / binCount};

    std::vector<float> angleHist(binCount, 0.f);
    for (int i = 0; i < angles.size(); i++) {
        int bin = std::clamp(static_cast<int>(floor(angles[i] / binRange)), 0,
                             binCount - 1);
        angleHist[bin] += weights[i];
    }

    // find modes of smoothed histogram
    std::vector<float> smoothedHist;
    std::vector<std::pair<float, int>> modes;
    findModesMeanShift(angleHist, smoothedHist, modes, 1);

    // if only one or no mode = > return invalid corner
    if (modes.size() <= 1)
        return;

    // compute orientation at modes and sort by angle
    float min_max[2];
    min_max[0] = modes[0].second * binRange;
    min_max[1] = modes[1].second * binRange;
    if (min_max[0] > min_max[1]) {
        float t = min_max[0];
        min_max[0] = min_max[1];
        min_max[1] = t;
    }

    const float deltaAngle = std::min(min_max[1] - min_max[0],
                                      min_max[0] - min_max[1] + (float)CV_PI);

    constexpr float kAngleThres{0.3f};
    if (deltaAngle <= kAngleThres)
        return;

    // set statistics: orientations
    m_cornersEdge1[index][0] = cos(min_max[0]);
    m_cornersEdge1[index][1] = sin(min_max[0]);
    m_cornersEdge2[index][0] = cos(min_max[1]);
    m_cornersEdge2[index][1] = sin(min_max[1]);
}

void MultiCheckerBoardDetector::Impl::findModesMeanShift(
    const std::vector<float> &hist, std::vector<float> &smoothed_hist,
    std::vector<std::pair<float, int>> &modes, float sigma)
{
    // efficient mean-shift approximation by histogram smoothing
    smoothed_hist.assign(hist.size(), 0.f);

    // compute smoothed histogram
    bool allZeros{true};
    const int length = static_cast<int>(hist.size());
    const int two_sigma = static_cast<int>(round(2 * sigma));
    for (int i{0}; i < length; i++) {
        float sum{0.f};
        for (int j{-two_sigma}; j <= two_sigma; j++) {
            sum += hist[(i + j) % length] *
                   normpdf(static_cast<float>(j), 0.f, sigma);
        }
        smoothed_hist[i] = sum;

        // TODO: use isApprox0
        // check if at least one entry is nonzero,
        // otherwise mode finding may run infinitly
        if (std::abs(smoothed_hist[i] - smoothed_hist[0]) > 0.0001)
            allZeros = false;
    }

    if (allZeros)
        return;

    // mode finding
    for (int i{0}; i < length; i++) {
        int cur = i;
        while (true) {
            int left = (cur - 1) % length;
            int right = (cur + 1) % length;
            float left_v = smoothed_hist[left];
            float right_v = smoothed_hist[right];
            float cur_v = smoothed_hist[cur];
            if (left_v >= cur_v && left_v >= right_v)
                cur = left;
            else if (right_v > cur_v && right_v > left_v)
                cur = right;
            else
                break;
        }

        bool isMax =
            modes.empty() ||
            std::none_of(modes.begin(), modes.end(), [&cur](const auto &pair) {
                return pair.second == cur;
            });

        if (isMax) {
            modes.push_back(std::make_pair(smoothed_hist[cur], cur));
        }
    }

    std::sort(modes.begin(), modes.end(),
              [](const auto &a, const auto &b) { return a.first > b.first; });
}

float MultiCheckerBoardDetector::Impl::scoreCorner(
    const cv::Mat &img, const cv::Mat &img_weight,
    const std::array<cv::Vec2f, 2> &edge_pair) const
{
    const cv::Vec2f center{img_weight.cols / 2.f, img_weight.cols / 2.f};

    // compute gradient filter kernel (bandwith = 3 px)
    const auto &e0 = edge_pair[0];
    const auto &e1 = edge_pair[1];
    cv::Matx22f ee0{e0[0], e0[0], e0[1], e0[1]};
    cv::Matx22f ee1{e1[0], e1[0], e1[1], e1[1]};

    cv::Mat img_filter = -cv::Mat::ones(img_weight.size(), img_weight.type());
    for (int x{0}; x < img_weight.cols; x++) {
        for (int y{0}; y < img_weight.rows; y++) {
            auto pt1 = cv::Vec2f(cv::Vec2i{x, y}) - center;
            auto pt2 = ee0 * pt1.mul(e0);
            auto pt3 = ee1 * pt1.mul(e1);
            double norm1 = cv::norm(pt1 - pt2);
            double norm2 = cv::norm(pt1 - pt3);

            constexpr float kFilterThres{1.5f};
            if (norm1 <= kFilterThres || norm2 <= kFilterThres) {
                img_filter.ptr<float>(y)[x] = 1.f;
            }
        }
    }

    auto normalizeImage = [](cv::Mat &img) {
        cv::Mat mean, stddev;
        cv::meanStdDev(img, mean, stddev);
        img.forEach<float>(
            [avg = mean.at<float>(0, 0), stdd = stddev.at<float>(0, 0)](
                float &val, const int * /*pos*/) { val = (val - avg) / stdd; });
    };

    cv::Mat img_weight_norm = img_weight.clone();
    normalizeImage(img_weight_norm);
    normalizeImage(img_filter);

    auto vectorizeMat = [](const cv::Mat &mat) -> std::vector<float> {
        std::vector<float> vec;
        vec.reserve(mat.size().area());
        mat.forEach<float>(
            [&vec](float &val, const int * /*pos*/) { vec.emplace_back(val); });
        return vec;
    };

    const auto vec_filter = vectorizeMat(img_filter);
    const auto vec_weight = vectorizeMat(img_weight);

    // compute gradient score
    float sum{0.};
    for (int i = 0; i < vec_weight.size(); i++) {
        sum += vec_weight[i] * vec_filter[i];
    }
    sum /= (vec_weight.size() - 1);
    float gradient_score = std::max(sum, 0.f);

    // create intensity filter kernel
    cv::Mat kerA, kerB, kerC, kerD;
    createKernels(atan2(e0[1], e0[0]), atan2(e1[1], e1[0]), img_weight.cols / 2,
                  kerA, kerB, kerC, kerD);

    // checkerboard responses
    double a1 = kerA.dot(img);
    double a2 = kerB.dot(img);
    double b1 = kerC.dot(img);
    double b2 = kerD.dot(img);

    double mu = (a1 + a2 + b1 + b2) / 4;
    double score_1 = std::min({mu - b2, mu - b1, a2 - mu, a1 - mu});
    double score_2 = std::min({b2 - mu, b1 - mu, mu - a2, mu - a1});
    double score_intensity = std::max({score_1, score_2, 0.});

    return static_cast<float>(gradient_score * score_intensity);
}

int MultiCheckerBoardDetector::Impl::directionalNeighbor(
    int idx, const cv::Vec2f &v, const cv::Mat &chessboard,
    const Corners &corners, int &neighbor_idx, float &min_dist) const
{
    // list of neighbor elements, which are currently not in use
    std::vector<int> used;
    used.reserve(chessboard.size().area());
    chessboard.forEach<int>([&used](int &val, const int * /*pos*/) {
        if (val != 0) {
            used.push_back(val);
        }
    });

    std::vector<int> unused(corners.pt.size());
    std::iota(unused.begin(), unused.end(), 0);
    for (auto it = unused.begin(); it != unused.end(); ++it) {
        for (const auto &use : used) {
            if (*it == use) {
                unused.erase(it);
                it--;
            }
        }
    }

    std::vector<float> dist_edge;
    std::vector<float> dist_point;

    cv::Vec2f pt_idx{corners.pt[idx].x, corners.pt[idx].y};
    // direction and distance to unused corners
    for (const auto &unuse : unused) {
        auto dir = cv::Vec2f{corners.pt[unuse].x, corners.pt[unuse].y} - pt_idx;
        auto dist = dir.dot(v);
        auto de = dir - dist * v;
        dist_edge.push_back(distance(de));
        dist_point.push_back(dist);
    }

    // find best neighbor
    int min_idx = 0;
    min_dist = std::numeric_limits<float>::max();

    // min_dist = dist_point[0] + 5 * dist_edge[0];
    for (int i = 0; i < dist_point.size(); i++) {
        if (dist_point[i] > 0) {
            float m = dist_point[i] + 5 * dist_edge[i];
            if (m < min_dist) {
                min_dist = m;
                min_idx = i;
            }
        }
    }
    neighbor_idx = unused[min_idx];

    return 1;
}

void MultiCheckerBoardDetector::Impl::predictCorners(
    const std::vector<cv::Vec2f> &p1, const std::vector<cv::Vec2f> &p2,
    const std::vector<cv::Vec2f> &p3, std::vector<cv::Vec2f> &pred) const
{
    CV_Assert(p1.size() == p2.size() && p1.size() == p3.size());

    pred.resize(p1.size());
    for (size_t i{0}; i < p1.size(); i++) {
        auto v1 = p2[i] - p1[i];
        auto v2 = p3[i] - p2[i];

        // predict angles
        auto a1 = atan2(v1[1], v1[0]);
        auto a2 = atan2(v1[1], v1[0]);
        auto a3 = 2.f * a2 - a1;

        // predict scales
        auto s1 = distance(v1);
        auto s2 = distance(v2);
        auto s3 = 2 * s2 - s1;
        pred[i] = p3[i] + 0.75 * s3 * cv::Vec2f{cosf(a3), sinf(a3)};
    }
}

void MultiCheckerBoardDetector::Impl::assignClosestCorners(
    const std::vector<cv::Vec2f> &cands, const std::vector<cv::Vec2f> &preds,
    std::vector<int> &indexes) const
{
    // return if not enough candidates are available
    if (cands.size() < preds.size()) {
        indexes.resize(1);
        indexes[0] = -1;
        return;
    }

    // build distance matrix
    cv::Mat dist_mat = cv::Mat::zeros(cands.size(), preds.size(), CV_32FC1);
    float min_dist = FLT_MAX;
    dist_mat.forEach<float>(
        [&cands, &preds, &min_dist](float &val, const int pos[]) {
            // pos: x, y (col, row)
            val = distance(cands[pos[1]], preds[pos[0]]);
            min_dist = std::min(min_dist, val);
        });

    // search greedily for closest corners
    indexes.resize(preds.size());
    for (int k = 0; k < preds.size(); k++) {
        bool doBreak{false};
        for (int row{0}; row < dist_mat.rows; row++) {
            for (int col{0}; col < dist_mat.cols; col++) {
                if (fabs(dist_mat.at<float>(row, col) - min_dist) < 10e-10) {
                    indexes[col] = row;
                    for (int c{0}; c < dist_mat.cols; c++) {
                        dist_mat.at<float>(row, c) = FLT_MAX;
                    }
                    for (int r{0}; r < dist_mat.rows; r++) {
                        dist_mat.at<float>(r, col) = FLT_MAX;
                    }

                    doBreak = true;
                    break;
                }
            }

            if (doBreak)
                break;
        }

        min_dist = FLT_MAX;
        dist_mat.forEach<float>([&min_dist](float &val, const int * /*pos*/) {
            min_dist = std::min(min_dist, val);
        });
    }
}

/// ------- MultiCheckerBoardDetector starts from here
MultiCheckerBoardDetector::MultiCheckerBoardDetector(const Options &opts)
    : d(std::make_unique<MultiCheckerBoardDetector::Impl>())
{
    d->init(opts);
}

MultiCheckerBoardDetector::~MultiCheckerBoardDetector() = default;

void MultiCheckerBoardDetector::setup(const Options &opts) { d->m_opts = opts; }

namespace {
constexpr int kTemplateCount{6};
constexpr int kRadii[]{4, 8, 12}; // 3 scales
const cv::Vec2f kTemplateProps[6]{
    {0.f, (float)CV_PI / 2}, {(float)CV_PI / 4, -(float)CV_PI / 4},
    {0.f, (float)CV_PI / 2}, {(float)CV_PI / 4, -(float)CV_PI / 4},
    {0.f, (float)CV_PI / 2}, {(float)CV_PI / 4, -(float)CV_PI / 4}};
} // namespace

void MultiCheckerBoardDetector::findCorners(const cv::Mat &img,
                                            Corners &corners)
{
    cv::Mat img_gray{img.size(), CV_8U};
    if (img.channels() == 1) {
        img.copyTo(img_gray);
    }
    else {
        cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    }

    // 0. Preprocess
    cv::GaussianBlur(img_gray, img_gray, cv::Size(9, 9), 1.5);

    cv::Mat img_norm;
    cv::normalize(img_gray, img_norm, 0, 1, cv::NORM_MINMAX, CV_32F);

    // 1. Find corners by 4 types of kernels
    cv::Mat imgCorners = cv::Mat::zeros(img_norm.size(), CV_32F);

    // #pragma omp parallel for num_threads(4)
    for (int i{0}; i < kTemplateCount; i++) {
        // a. create 4 kernels
        cv::Mat kerA, kerB, kerC, kerD;
        d->createKernels(kTemplateProps[i][0], kTemplateProps[i][1],
                         kRadii[i / 2], kerA, kerB, kerC, kerD);

        // b. calculate response to each kernels
        cv::Mat imgCornerA1, imgCornerB1, imgCornerC1, imgCornerD1;
        cv::filter2D(img_norm, imgCornerA1, CV_32F, kerA); // a1
        cv::filter2D(img_norm, imgCornerB1, CV_32F, kerB); // a2
        cv::filter2D(img_norm, imgCornerC1, CV_32F, kerC); // b1
        cv::filter2D(img_norm, imgCornerD1, CV_32F, kerD); // b2

        cv::Mat imgCornerMean =
            (imgCornerA1 + imgCornerB1 + imgCornerC1 + imgCornerD1) / 4.f;

        cv::Mat imgCornerA, imgCornerB, imgCorner1, imgCorner2;
        // case 1: a = white, b = black
        cv::min(cv::Mat(imgCornerA1 - imgCornerMean),
                imgCornerB1 - imgCornerMean, imgCornerA);
        cv::min(cv::Mat(imgCornerMean - imgCornerC1),
                imgCornerMean - imgCornerD1, imgCornerB);
        cv::min(imgCornerA, imgCornerB, imgCorner1);
        // case 2: b = white, a = black
        cv::min(cv::Mat(imgCornerMean - imgCornerA1),
                imgCornerMean - imgCornerB1, imgCornerA);
        cv::min(cv::Mat(imgCornerC1 - imgCornerMean),
                imgCornerD1 - imgCornerMean, imgCornerB);
        cv::min(imgCornerA, imgCornerB, imgCorner2);

        // update corner map
        cv::max(imgCorners, imgCorner1, imgCorners);
        cv::max(imgCorners, imgCorner2, imgCorners);
    }

    // 2. Extract corner candidates via non maximum suppression
    std::vector<cv::Point2f> raw_corners;
    d->nonMaximumSuppression(imgCorners, raw_corners, 3, 5, 0.025f);
    d->resizeCornersEdge(raw_corners.size());

    // 3. Subpixel refinement
    cv::Mat img_du, img_dv, img_angle, img_weight;
    calcImageAngleAndWeight(img_norm, img_du, img_dv, img_angle, img_weight);

    if (d->m_opts.doSubPix) {
        d->refineCorners(raw_corners, img_du, img_dv, img_angle, img_weight,
                         10);

        // remove corners without edges
        for (size_t i{0}; i < raw_corners.size(); i++) {
            if (d->m_cornersEdge1[i] == cv::Vec2f{} &&
                d->m_cornersEdge2[i] == cv::Vec2f{}) {
                raw_corners[i] = {};
            }
        }
    }

    // The following steps are for chessboard growing
    // 4. score corners
    std::vector<float> score;
    const std::vector<int> radii{std::begin(kRadii), std::end(kRadii)};
    d->scoreCorners(img_norm, img_weight, raw_corners, radii, score);

    for (size_t i{0}; i < raw_corners.size(); i++) {
        if (score[i] > d->m_opts.scoreThreshold) {
            corners.pt.push_back(raw_corners[i]);
            corners.v1.push_back(d->m_cornersEdge1[i]);
            corners.v2.push_back(d->m_cornersEdge2[i]);
            corners.score.push_back(score[i]);
        }
    }

    std::vector<cv::Vec2f> corners_n1(corners.pt.size());
    for (size_t i{0}; i < corners_n1.size(); i++) {
        if (corners.v1[i][0] + corners.v1[i][1] < 0.f) {
            corners.v1[i] = -corners.v1[i];
        }

        float flipflag = corners.v1[i][0] * corners.v2[i][0] +
                         corners.v1[0][1] * corners.v2[i][1];
        if (flipflag > 0)
            flipflag = -1.f;
        else
            flipflag = 1.f;
        corners.v2[i] = flipflag * corners.v2[i];
    }
}

void MultiCheckerBoardDetector::chessboardsFromCorners(
    const Corners &corners, std::vector<cv::Mat> &chessboards)
{
    for (int i = 0; i < corners.pt.size(); i++) {
        cv::Mat chessboard_i = d->initChessboard(corners, i);
        if (chessboard_i.empty()) {
            continue;
        }

        float board_energy = d->chessboardEnergy(chessboard_i, corners);
        if (board_energy > 0.f) {
            continue;
        }

        int s = 0;
        // try growing chessboard
        while (true) {
            s++;
            // a. compute current energy
            float energy = d->chessboardEnergy(d->m_chessboard, corners);

            // b. compute proposals and energies
            constexpr int kProposeCount{4};
            std::vector<cv::Mat> prop_boards(kProposeCount);
            std::vector<float> prop_energies(kProposeCount);
            for (int j{0}; j < kProposeCount; j++) {
                prop_boards[j] = d->growChessboard(d->m_chessboard, corners, j);
                prop_energies[j] = d->chessboardEnergy(prop_boards[j], corners);
            }

            // c. find best proposal
            auto min =
                std::min_element(prop_energies.begin(), prop_energies.end());
            int min_idx = std::distance(prop_energies.begin(), min);

            // d. accept proposal if energy get smaller
            if (*min >= energy) {
                break;
            }

            d->m_chessboard = prop_boards[min_idx].clone();
        }

        if (d->chessboardEnergy(d->m_chessboard, corners) < -10) {
            // check if new chessboard overlaps with existing chessboards
            cv::Mat overlap = cv::Mat::zeros(chessboards.size(), 2, CV_32FC1);
            for (size_t j{0}; j < chessboards.size(); j++) {
                bool isbreak = false;
                for (int k{0}; k < chessboards[j].size().area(); k++) {
                    int refv = chessboards[j].at<int>(k / chessboards[j].cols,
                                                      k % chessboards[j].cols);
                    for (int l{0}; l < d->m_chessboard.size().area(); l++) {
                        int isv = d->m_chessboard.at<int>(
                            l / d->m_chessboard.cols, l % d->m_chessboard.cols);
                        if (refv == isv) {
                            overlap.at<float>(j, 0) = 1.f;
                            overlap.at<float>(j, 1) =
                                d->chessboardEnergy(chessboards[j], corners);
                            isbreak = true;
                            break;
                        }
                    }
                }
            }

            // add chessboard(and replace overlapping if neccessary)
            cv::Mat_<float> overlaps = overlap.col(0);
            bool isOverlap = std::any_of(overlaps.begin(), overlaps.end(),
                                         [](const float &o) {
                                             // TODO: isApprox
                                             return std::abs(o) == 1.f;
                                         });

            if (!isOverlap) {
                chessboards.push_back(d->m_chessboard);
            }
            else {
                bool flagpush{true};
                std::vector<bool> flagerase(overlap.rows, false);
                float ce = d->chessboardEnergy(d->m_chessboard, corners);
                for (int i1 = 0; i1 < overlap.rows; i1++) {
                    if (fabs(overlap.at<float>(i1, 0)) > 0.0001) {
                        bool isb1 = overlap.at<float>(i1, 1) > ce;

                        int a = int(overlap.at<float>(i1, 1) * 1000);
                        int b = int(ce * 1000);

                        bool isb2 = a > b;
                        if (isb1 != isb2)
                            printf("find bug!\n");

                        if (isb2) {
                            flagerase[i1] = true;
                        }
                        else {
                            flagpush = false;
                            //	break;
                        }
                    }
                }

                if (flagpush) {
                    for (int i1 = 0; i1 < chessboards.size();) {
                        auto it = chessboards.begin() + i1;
                        auto it1 = flagerase.begin() + i1;
                        if (*it1) {
                            chessboards.erase(it);
                            flagerase.erase(it1);
                            i1 = 0;
                        }
                        i1++;
                    }
                    chessboards.push_back(d->m_chessboard);
                }
            }
        }
    }
}

void MultiCheckerBoardDetector::drawChessboards(
    cv::Mat &img, const std::vector<cv::Mat> &chessboards,
    const Corners &corners) const
{
    if (img.channels() == 1) {
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    }

    int n = 8;
    if (img.rows < 2000 || img.cols < 2000) {
        n = 2;
    }

    cv::RNG rng(0xFFFFFFFF);
    for (const auto &chessboard : chessboards) {
        cv::Scalar color(rng.uniform(0.0, 1.0), rng.uniform(0.0, 1.0),
                         rng.uniform(0.0, 1.0));
        color = color * 255;

        chessboard.forEach<int>(
            [&img, &corners, &color, &n](int &val, const int * /*pos*/) {
                cv::circle(img, cv::Point(corners.pt[val]), n, color, n);
            });
    }
}

} // namespace tl
