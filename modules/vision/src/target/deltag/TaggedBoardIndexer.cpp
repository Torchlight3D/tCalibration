#include "TaggedBoardIndexer.h"

#include <sstream>
#include <string>

#include "GrayModel.h"
#include "Quadangle.h"

#include "Triangle.h"

#include <deltille/apriltags/Tag16h5.h>
#include <deltille/apriltags/Tag25h7.h>
#include <deltille/apriltags/Tag25h9.h>
#include <deltille/apriltags/Tag36h9.h>
#include <deltille/apriltags/Tag36h11.h>
#include <deltille/deltags/Tag16h5.h>
#include <deltille/deltags/Tag25h7.h>
#include <deltille/deltags/Tag25h9.h>
#include <deltille/deltags/Tag36h9.h>
#include <deltille/deltags/Tag36h11.h>

namespace orp {
namespace calibration {

using DelTag = Triangle<cv::Point2f>;
using AprilTag = Quadangle<cv::Point2f>;

const Deltille::TagCodes &tagFamilyNameToCodes(const std::string &family)
{
    if (family == "aprilTag16h5")
        return Deltille::aprilTagCodes16h5;
    else if (family == "aprilTag25h7")
        return Deltille::aprilTagCodes25h7;
    else if (family == "aprilTag25h9")
        return Deltille::aprilTagCodes25h9;
    else if (family == "aprilTag36h9")
        return Deltille::aprilTagCodes36h9;
    else if (family == "aprilTag36h11")
        return Deltille::aprilTagCodes36h11;
    else if (family == "delTag16h5")
        return Deltille::delTagCodes16h5;
    else if (family == "delTag25h7")
        return Deltille::delTagCodes25h7;
    else if (family == "delTag25h9")
        return Deltille::delTagCodes25h9;
    else if (family == "delTag36h9")
        return Deltille::delTagCodes36h9;
    else if (family == "delTag36h11")
        return Deltille::delTagCodes36h11;
    else {
        printf("Unknown family %s, defaulting to aprilTag36h11\n",
               family.c_str());
        return Deltille::aprilTagCodes36h11;
    }
}

void writeBoardObservations(const char *filename,
                            const std::vector<BoardDefinition> &defs,
                            const std::vector<BoardObservation> &boards)
{
    FILE *out = fopen(filename, "w");
    if (out != NULL) {
        for (size_t b = 0; b < boards.size(); ++b) {
            auto &board = boards[b];
            auto &def = defs[board.board_id];
            auto &obs_corners = board.corner_locations;
            for (size_t j = 0; j < obs_corners.size(); j++) {
                if (defs.size() > 0) {
                    int r = int(j / def.cols), c = int(j % def.cols);
                    auto &pt = def.corner_locations.at<cv::Vec3d>(r, c);
                    fprintf(out, "%d %f %f %f %f %f\n", board.board_id,
                            obs_corners[j].x, obs_corners[j].y, pt[0], pt[1],
                            pt[2]);
                }
                else
                    fprintf(out, "%f %f\n", obs_corners[j].x, obs_corners[j].y);
            }
        }
        fclose(out);
    }
}

bool updateBoardsWithCalibratedTargetFile(std::istream &in,
                                          std::vector<BoardDefinition> &defs)
{
    // updates x,y,z locations of points on boards using calibration target
    // calibration data
    std::string line;
    std::istringstream iss;
    int lineno = 1;

    std::getline(in, line);
    iss.str(line);
    iss.clear();
    int npattern = -1;
    if (!(iss >> npattern) || npattern != (int)defs.size()) {
        printf("updateBoardsWithCalibratedTargetFile: mismatching number of "
               "boards "
               "on the target (defs: %d vs file: %d)!\n",
               int(defs.size()), npattern);
        return false;
    }
    for (int i = 0; i < npattern; ++i) {
        // BoardDefinition &def = defs[i];
        cv::Mat &pts = defs[i].corner_locations;
        int npoints = -1;
        ++lineno;
        std::getline(in, line);
        iss.str(line);
        iss.clear();
        ;
        if (!(iss >> npoints) || npoints != pts.size().area()) {
            printf(
                "updateBoardsWithCalibratedTargetFile: board %d, mismatching "
                "number of points on the board (defs: %d vs file: %d)!\n",
                i, int(pts.size().area()), npoints);
            return false;
        }
        double max_dist = 0;
        // diagonal/side of the board?
        double sz = cv::norm(pts.at<cv::Vec3d>(0, 0) -
                             pts.at<cv::Vec3d>(pts.rows - 1, pts.cols - 1));
        for (int r = 0; r < pts.rows; ++r)
            for (int c = 0; c < pts.cols; ++c) {
                ++lineno;
                std::getline(in, line);
                iss.str(line);
                iss.clear();
                double x, y, z;
                if (!(iss >> x >> y >> z))
                    printf("updateBoardsWithCalibratedTargetFile: board %d, "
                           "invalid "
                           "point on line: %d\n>%s<\n",
                           i, lineno, line.c_str());

                cv::Vec3d pt(x, y, z);
                cv::Vec3d &old_pt = pts.at<cv::Vec3d>(r, c);
                double dist = cv::norm(pt - old_pt);
                if (dist > max_dist)
                    max_dist = dist;
                old_pt = pt;
            }
        if (max_dist > sz / 10.0)
            printf("updateBoardsWithCalibratedTargetFile: board %d, diagonal "
                   "length: "
                   "%f, max. corrected point distance: %f\n",
                   i, sz, max_dist);
    }

    return true;
}

void readBoardDefinitions(std::istream &in, std::vector<BoardDefinition> &defs,
                          TagFamilies &detectors)
{
    std::string line;
    std::map<int, BoardDefinition> boards;
    auto cur = boards.end();

    // generate new target_id
    int target_id = 0;
    if (defs.size() > 0)
        target_id = defs.back().target_id + 1;

    while (std::getline(in, line)) {
        std::istringstream iss(line);
        int board, tagid, lx, ly;
        double x, y, z;
        float bbits;
        char c;
        if (!(iss >> tagid >> c >> lx >> c >> ly >> c >> x >> c >> y >> c >>
              z)) {
            // If not a corner, read two lines of header
            iss.str(line);
            iss.clear();
            // First line: board_id, cols, rows, size
            if (!(iss >> board >> c >> lx >> c >> ly >> c >> x)) {
                printf("Error reading file, line >%s<\n", line.c_str());
                break;
            }
            else {
                BoardDefinition b;
                b.id = board;
                b.target_id = target_id;
                b.cols = lx;
                b.rows = ly;
                b.size = x;
                b.corner_locations.create(b.rows, b.cols, CV_64FC3);

                // Second line: family_name, border_bits
                std::getline(in, b.family, ',');
                std::getline(in, line);
                iss.str(line);
                iss.clear();
                iss >> bbits;
                b.border_bits = bbits;

                // Determine checkerboard/deltille from tag family name
                if (b.family.compare(0, 8, "aprilTag") == 0) {
                    b.triangular = false;
                }
                else if (b.family.compare(0, 6, "delTag") == 0) {
                    b.triangular = true;
                }
                else {
                    throw std::runtime_error("unknown tag family: " + b.family);
                }

                // Todo: Replace the key with {family, bbits}
                if (detectors.count(b.family) == 0) {
                    std::shared_ptr<TagFamily> det =
                        std::make_shared<TagFamily>(
                            tagFamilyNameToCodes(b.family), b.border_bits);
                    printf("Loading detector for tag family: >%s<\n",
                           b.family.c_str());
                    auto detector =
                        detectors.insert(std::make_pair(b.family, det));
                    if (detector.second)
                        b.detector = det;
                }
                else
                    b.detector = detectors[b.family];
                auto tmp = boards.insert(std::make_pair(board, b));
                if (tmp.second)
                    cur = tmp.first;
            }
        }
        else {
            // the points will be in the matrix indexed from top to bottom
            // (unlike on the target), flip them upside down here
            cur->second.corner_locations.at<cv::Vec3d>(ly, lx) =
                cv::Vec3d(x, y, z);

            if (tagid < 0)
                continue;

            cur->second.tag_locations.insert(
                std::make_pair(tagid, cv::Point(lx, ly)));
        }
    }

    int unique_id_idx = 0;
    for (auto &&det : detectors) {
        // compute offsets of ids in each detector
        det.second->tagFamilyOffset = unique_id_idx;
        unique_id_idx += int(det.second->codes.size());
    }

    for (auto &&t : boards) defs.push_back(t.second);
}

static inline bool getTriangleBitCoordinate(const DelTag &t, float dd,
                                            bool lower, int width, int height,
                                            float x, float y, float &rx,
                                            float &ry, int &imx, int &imy)
{
    // if bit 0 corner is at 0,0 of the triangle coordinate system, then
    // centroid of lower triangle is at 1/3, 1/3 if bit 0 corner is at 0,0 of
    // the triangle coordinate system, then centroid of upper triangle is at
    // 2/3, 2/3
    float xc, yc;
    if (lower)
        xc = yc = 1.0f / 3.0f;
    else
        xc = yc = 2.0f / 3.0f;
    rx = (xc + x) / dd;
    ry = (yc + y) / dd;
    DelTag::point_type pxy = t.interpolate01(rx, ry);
    imx = (int)((pxy.x + 0.5));
    imy = (int)((pxy.y + 0.5));
    return (imx >= 0 && imx < width && imy >= 0 && imy < height);
}

template <typename PixelType>
TagDetection identifyTagFromTriangle(const cv::Mat &img,
                                     const TagFamily &tag_family, DelTag &t)
{
    int width = img.cols, height = img.rows;
    // Find a threshold using local white and black model...
    GrayModel<float> blackModel, whiteModel;

    // number of bits in triangle = 2 * (border + 1) + dimension of the tag  +1
    const float dd = 3 * tag_family.blackBorder + tag_family.dimension;

    // assume at least one bit black border, and triangle with base vector 0,1
    // traverse all three edges at once...
    const int ddi = int(dd);
    for (int i = -1; i <= ddi; i++) {
        // outer "white" bit reside in -1,-1 bit/edge upper triangles...
        int irx, iry;
        float rx, ry;
        if (getTriangleBitCoordinate(t, dd, false, width, height, float(i), -1,
                                     rx, ry, irx, iry))
            whiteModel.add(rx, ry, img.at<PixelType>(iry, irx));
        if (getTriangleBitCoordinate(t, dd, false, width, height, -1, float(i),
                                     rx, ry, irx, iry))
            whiteModel.add(rx, ry, img.at<PixelType>(iry, irx));
        // the diagonal are in upper part of -1, dd - 1
        if (getTriangleBitCoordinate(t, dd, false, width, height, dd - i - 1,
                                     float(i), rx, ry, irx, iry))
            whiteModel.add(rx, ry, img.at<PixelType>(iry, irx));
        // inner, black bits are on lower triangles of bits 0, 0
        if (i >= 0 && i < ddi) {
            if (getTriangleBitCoordinate(t, dd, true, width, height, float(i),
                                         -0, rx, ry, irx, iry))
                blackModel.add(rx, ry, img.at<PixelType>(iry, irx));
            if (getTriangleBitCoordinate(t, dd, true, width, height, 0,
                                         float(i), rx, ry, irx, iry))
                blackModel.add(rx, ry, img.at<PixelType>(iry, irx));
            if (getTriangleBitCoordinate(t, dd, true, width, height, dd - i - 1,
                                         float(i), rx, ry, irx, iry))
                blackModel.add(rx, ry, img.at<PixelType>(iry, irx));
        }
    }
    // ok, lets look at the tag bits...
    //
    // the geometry of the tag is as follows:
    //   1. in each row R we has odd number of bits only D - R lower bits and D
    //   - R - 1 upper bits
    //   2. bits are alternating in lower and upper triangles
    //   3. tag is rendered from top to bottom => row D-1 first has lowest y
    //   coordinate
    bool bad = false;
    unsigned long long tag_code = 0;
    for (int r = tag_family.dimension - 1; r >= 0; --r) {
        int row_bits = tag_family.dimension - r;
        float b = tag_family.blackBorder;
        for (int c = 0; c < row_bits; ++c) {
            int irx, iry;
            float rx, ry;
            tag_code <<= 1;
            if (getTriangleBitCoordinate(t, dd, true, width, height, b + c,
                                         b + r, rx, ry, irx, iry)) {
                float threshold = (blackModel.interpolate(rx, ry) +
                                   whiteModel.interpolate(rx, ry)) *
                                  0.5f;
                if (img.at<PixelType>(iry, irx) > threshold)
                    tag_code |= 1;
            }
            else
                bad = true;
            if (c == row_bits - 1)
                continue;
            tag_code <<= 1;
            if (getTriangleBitCoordinate(t, dd, false, width, height, b + c,
                                         b + r, rx, ry, irx, iry)) {
                float threshold = (blackModel.interpolate(rx, ry) +
                                   whiteModel.interpolate(rx, ry)) *
                                  0.5f;
                if (img.at<PixelType>(iry, irx) > threshold)
                    tag_code |= 1;
            }
            else
                bad = true;
        }
    }
    TagDetection det;
    det.good = !bad;
    if (!bad)
        tag_family.decodeTritag(det, tag_code);
    return det;
}

static inline bool getQuadBitCoordinate(AprilTag &q, float dd, int width,
                                        int height, float x, float y, float &rx,
                                        float &ry, int &imx, int &imy)
{
    rx = (x + 0.5f) / dd;
    ry = (y + 0.5f) / dd;
    AprilTag::point_type pxy = q.interpolate01(rx, ry);
    imx = (int)((pxy.x + 0.5));
    imy = (int)((pxy.y + 0.5));
    return (imx >= 0 && imx < width && imy >= 0 && imy < height);
}

template <typename PixelType>
TagDetection identifyTagFromQuad(const cv::Mat &img,
                                 const TagFamily &tag_family, AprilTag &q)
{
    int width = img.cols, height = img.rows;
    // Find a threshold
    GrayModel<float> blackModel, whiteModel;
    const float dd = 2 * tag_family.blackBorder + tag_family.dimension;
    for (int idx = 1; idx < dd; ++idx) {
        int irx, iry;
        float rx, ry;
        // use "bits" around the quad as white samples
        if (getQuadBitCoordinate(q, dd, width, height, float(idx), -1.f, rx, ry,
                                 irx, iry))
            whiteModel.add(rx, ry, img.at<PixelType>(iry, irx));
        if (getQuadBitCoordinate(q, dd, width, height, -1.f, float(idx), rx, ry,
                                 irx, iry))
            whiteModel.add(rx, ry, img.at<PixelType>(iry, irx));
        if (getQuadBitCoordinate(q, dd, width, height, float(idx), dd, rx, ry,
                                 irx, iry))
            whiteModel.add(rx, ry, img.at<PixelType>(iry, irx));
        if (getQuadBitCoordinate(q, dd, width, height, dd, float(idx), rx, ry,
                                 irx, iry))
            whiteModel.add(rx, ry, img.at<PixelType>(iry, irx));

        // use "bits" inside the quad as black samples
        if (getQuadBitCoordinate(q, dd, width, height, float(idx), 0, rx, ry,
                                 irx, iry))
            blackModel.add(rx, ry, img.at<PixelType>(iry, irx));
        if (getQuadBitCoordinate(q, dd, width, height, 0, float(idx), rx, ry,
                                 irx, iry))
            blackModel.add(rx, ry, img.at<PixelType>(iry, irx));
        if (getQuadBitCoordinate(q, dd, width, height, float(idx), dd - 1, rx,
                                 ry, irx, iry))
            blackModel.add(rx, ry, img.at<PixelType>(iry, irx));
        if (getQuadBitCoordinate(q, dd, width, height, dd - 1, float(idx), rx,
                                 ry, irx, iry))
            blackModel.add(rx, ry, img.at<PixelType>(iry, irx));
    }
    bool bad = false;
    unsigned long long tag_code = 0;
    for (int iy = tag_family.dimension - 1; iy >= 0; iy--) {
        for (int ix = 0; ix < tag_family.dimension; ix++) {
            int irx, iry;
            float rx, ry;
            tag_code <<= 1;
            if (getQuadBitCoordinate(
                    q, dd, width, height, tag_family.blackBorder + ix,
                    tag_family.blackBorder + iy, rx, ry, irx, iry)) {
                float threshold = (blackModel.interpolate(rx, ry) +
                                   whiteModel.interpolate(rx, ry)) *
                                  0.5f;
                if (img.at<PixelType>(iry, irx) > threshold)
                    tag_code |= 1;
            }
            else
                bad = true;
        }
    }
    TagDetection det;
    det.good = !bad;
    if (!bad)
        tag_family.decode(det, tag_code);
    return det;
}

bool fixFullCheckerBoardOrientation(const cv::Mat &img,
                                    const cv::Size &board_size,
                                    BoardObservation &board)
{
    bool foundOrientation = false;
    if (board_size.area() > 0) {
        cv::Mat &best_checkerboard = board.board;
        std::vector<cv::Point2f> &pts = board.corner_locations;

        // find the right orientation
        if (best_checkerboard.rows == board_size.height &&
            best_checkerboard.cols ==
                board_size.width) // when both sides are confirmed.
        {
            if ((board_size.height + board_size.width) % 2 == 1) {
                // when size is even and odd, sample center pixels
                cv::Mat Imap = cv::Mat(best_checkerboard.rows - 1,
                                       best_checkerboard.cols - 1, CV_64FC1);
                for (int r = 0; r < best_checkerboard.rows - 1; ++r) {
                    for (int c = 0; c < best_checkerboard.cols - 1; ++c) {
                        int id1, id2;
                        if (best_checkerboard.at<int>(r, c) > -1 &&
                            best_checkerboard.at<int>(r + 1, c + 1) > -1) {
                            id1 = best_checkerboard.at<int>(r, c);
                            id2 = best_checkerboard.at<int>(r + 1, c + 1);
                        }
                        else if (best_checkerboard.at<int>(r, c + 1) > -1 &&
                                 best_checkerboard.at<int>(r + 1, c) > -1) {
                            id1 = best_checkerboard.at<int>(r, c + 1);
                            id2 = best_checkerboard.at<int>(r + 1, c);
                        }
                        else {
                            Imap.at<double>(r, c) = -1;
                            continue;
                        }
                        // bilinear sample intensity at the centroid of square
                        double x = (pts[id1].x + pts[id2].x) / 2.0;
                        double y = (pts[id1].y + pts[id2].y) / 2.0;
                        double I =
                            (img.at<uint8_t>(int(y), int(x)) +
                             img.at<uint8_t>(int(y), int(x + 0.5)) +
                             img.at<uint8_t>(int(y + 0.5), int(x)) +
                             img.at<uint8_t>(int(y + 0.5), int(x + 0.5))) /
                            4.0;
                        Imap.at<double>(r, c) = I;
                    }
                }

                // you can change the strategy here.
                double black = 0, white = 0;
                int num_black = 0, num_white = 0;
                for (int r = 0; r < best_checkerboard.rows - 1; ++r) {
                    for (int c = 0; c < best_checkerboard.cols - 1; ++c) {
                        if ((r + c) % 2 == 0) { // black
                            if (Imap.at<double>(r, c) > -1) {
                                black += Imap.at<double>(r, c);
                                ++num_black;
                            }
                        }
                        else { // white
                            if (Imap.at<double>(r, c) > -1) {
                                white += Imap.at<double>(r, c);
                                ++num_white;
                            }
                        }
                    }
                }
                if (num_black > 0 && num_white > 0) {
                    if (black / num_black >
                        white / num_white) { // if black is brighter, flip the
                                             // checkerboard
                        // cout << "FLIP !" << endl;
                        cv::Mat tmp = best_checkerboard.clone();
                        for (int r = 0; r < best_checkerboard.rows; ++r)
                            for (int c = 0; c < best_checkerboard.cols; ++c)
                                best_checkerboard.at<int>(r, c) =
                                    tmp.at<int>(best_checkerboard.rows - 1 - r,
                                                best_checkerboard.cols - 1 - c);
                    }
                }
                foundOrientation = true;
            }
        }
        else { // span to a plausible checkeboard
            cv::Mat best_checkerboard_fullsize = cv::Mat(
                board_size.height, board_size.width, CV_32S, cv::Scalar(-1));
            for (int r = 0; r < best_checkerboard.rows; ++r)
                for (int c = 0; c < best_checkerboard.cols; ++c)
                    best_checkerboard_fullsize.at<int>(r, c) =
                        best_checkerboard.at<int>(r, c);
            best_checkerboard = best_checkerboard_fullsize.clone();
        }

        // finally re-order corners...
        std::vector<cv::Point2f> reordered(best_checkerboard.size().area(),
                                           cv::Point2f(-1, -1));
        int cnt = 0;
        for (int r = 0; r < best_checkerboard.rows; ++r)
            for (int c = 0; c < best_checkerboard.cols; ++c) {
                if (best_checkerboard.at<int>(r, c) != -1) {
                    reordered[cnt] =
                        board.corner_locations[best_checkerboard.at<int>(r, c)];
                    best_checkerboard.at<int>(r, c) = cnt;
                }
                ++cnt;
            }
        board.indexed = true;
        board.corner_locations = reordered;
    }
    return foundOrientation;
}

int fixFullCheckerBoardOrientations(const cv::Mat &img,
                                    const cv::Size &board_size,
                                    std::vector<BoardObservation> &boards)
{
    int boards_fixed = 0;
    for (size_t b = 0; b < boards.size(); ++b)
        if (fixFullCheckerBoardOrientation(img, board_size, boards[b]))
            ++boards_fixed;
    return boards_fixed;
}

void TaggedBoardIndexer::fixCheckerBoards(const cv::Mat &img,
                                          std::vector<BoardObservation> &boards)
{
    for (size_t b = 0; b < boards.size(); ++b) {
        std::vector<int> tags;
        // form Quads out of each board and
        BoardObservation &obs = boards[b];
        std::vector<cv::Point2f> p(4);
        obs.tags.resize(obs.corner_locations.size());
        for (int r = 0; r < obs.board.rows - 1; ++r) {
            for (int c = 0; c < obs.board.cols - 1; ++c) {
                cv::Mat &m = obs.board;
                std::vector<TagDetection> &t = obs.tags;
                std::vector<cv::Point2f> &cor = obs.corner_locations;
                // for each complete quad...
                if (m.at<int>(r, c) != -1 && m.at<int>(r, c + 1) != -1 &&
                    m.at<int>(r + 1, c) != -1 &&
                    m.at<int>(r + 1, c + 1) != -1) {
                    p[0] = cor[m.at<int>(r, c)];
                    p[1] = cor[m.at<int>(r, c + 1)];
                    p[2] = cor[m.at<int>(r + 1, c + 1)];
                    p[3] = cor[m.at<int>(r + 1, c)];

                    TagDetection det;
                    std::shared_ptr<TagFamily> best_detector;
                    for (auto &&d : detectors) {
                        AprilTag tag(p);
                        det =
                            identifyTagFromQuad<uint8_t>(img, *(d.second), tag);
                        if (det.good && det.hammingDistance <= 2) {
                            best_detector = d.second;
                            break;
                        }
                    }

                    if (det.good) {
                        // localize tag on boards...
                        auto tag = tag_to_board_map.find(
                            det.id + best_detector->tagFamilyOffset);
                        if (tag != tag_to_board_map.end()) {
                            // repurpose some of the variables, to store board
                            // id, and position
                            det.hammingDistance = tag->second.first;
                            det.code = tag->second.second.y;    // row
                            det.obsCode = tag->second.second.x; // column

                            // when a rotation is detected, make sure the QR tag
                            // is associated with the correct corner (bottom,
                            // left) *do not change the rotation at this point
                            int realr = r, realc = c;
                            switch (det.rotation) {
                                case 1:
                                    realc++;
                                    break;
                                case 2:
                                    realr++;
                                    realc++;
                                    break;
                                case 3:
                                    realr++;
                                    break;
                            }

#ifdef DEBUG_REINDEXING
                            printf("FOUND tag: %d, fam: %s, detection: %d, "
                                   "rotation: %d, "
                                   "position r: %d, c:%d from board: %d (%d of "
                                   "target %d), "
                                   "at location r:%d, c:%d\n",
                                   det.id, det.tagFamily,
                                   m.at<int>(realr, realc), det.rotation,
                                   tag->second.second.y, tag->second.second.x,
                                   tag->second.first,
                                   board_defs[tag->second.first].id,
                                   board_defs[tag->second.first].target_id,
                                   realr, realc);
#endif
                            tags.push_back(m.at<int>(realr, realc));
                            t[m.at<int>(realr, realc)] = det;
                        }
                        else {
                            // unknown tag... ignore it...
                            det.id = -1;
                            det.good = false;
                        }
                    }
                    else
                        det.id = -1;
                }
            }
        }

        std::vector<int> rot(4, 0);
        std::vector<int> bid(board_defs.size(), 0);
        // now check if tags have consistent rotation and board id...
        for (size_t t = 0; t < tags.size(); ++t) {
            TagDetection &det = obs.tags[tags[t]];
            rot[det.rotation]++;
            bid[det.hammingDistance]++;
        }
        int best = -1, mx = 0, inc = 0;
        for (size_t r = 0; r < bid.size(); ++r) {
            if (bid[r] > mx) {
                best = int(r);
                if (best != -1) // count inconsistent observations of board ids
                    inc += mx;
                mx = bid[r];
            }
            else
                inc += bid[r];
        }
        if (best == -1)
            continue;
        if (inc > 0) {
            printf("Inconsistent board ids, best: %d votes, inconsistent: %d\n",
                   mx, inc);
            printf("Histogram: ");
            for (size_t r = 0; r < bid.size(); ++r)
                printf("[%d: %d] ", int(r), bid[r]);
            printf("\n");
            continue;
        }
        obs.board_id = best;
        best = -1, mx = 0, inc = 0;
        for (size_t r = 0; r < rot.size(); ++r) {
            if (rot[r] > mx) {
                best = int(r);
                if (best != -1) // count inconsistent observations of rotations
                    inc += mx;
                mx = rot[r];
            }
            else
                inc = +rot[r];
        }
        if (best == -1)
            continue;
        if (inc > 0) {
            printf("Inconsistent tag orientations, best: %d votes, "
                   "inconsistent: %d.\n",
                   mx, inc);
            continue;
        }

        cv::Mat rotated = obs.board.clone();
        // rotate board according to the tag...
        switch (best) {
            case 1:
                transpose(rotated, rotated);
                flip(rotated, rotated, 0); // transpose+flip(1)=CW
                break;
            case 2:
                flip(rotated, rotated, -1); // flip(-1)=180
                break;
            case 3:
                transpose(rotated, rotated);
                flip(rotated, rotated, 1); // transpose+flip(1)=CW
                break;
            default:
                break;
        }
#ifdef DEBUG_REINDEXING
        printf("Consistent observation of board %d (%d), size %d x %d, "
               "orientation: %d\n",
               board_defs[obs.board_id].id, obs.board_id,
               board_defs[obs.board_id].rows, board_defs[obs.board_id].cols,
               best * 90);
#if DEBUG_REINDEXING == 3
        std::cout << "Detected:" << std::endl;
        std::cout << obs.board << std::endl;
        std::cout << "Rotated:" << std::endl;
        std::cout << rotated << std::endl;
#endif
#endif
        // now find proper offset...
        std::map<int, int> ofsx, ofsy;
        for (int r = 0; r < rotated.rows; ++r)
            for (int c = 0; c < rotated.cols; ++c) {
                if (rotated.at<int>(r, c) != -1 &&
                    obs.tags[rotated.at<int>(r, c)].good) {
                    TagDetection &det = obs.tags[rotated.at<int>(r, c)];
                    int ox = int(c - det.obsCode);
                    int oy = int(r - det.code);
#if DEBUG_REINDEXING == 3
                    printf("Tag: %d, detection: %d, position r:%d, c:%d at "
                           "location "
                           "r:%d, %d\n",
                           det.id, rotated.at<int>(r, c), int(det.code),
                           int(det.obsCode), r, c);
#endif
                    auto itx = ofsx.find(ox), ity = ofsy.find(oy);
                    if (itx != ofsx.end())
                        ofsx[ox]++;
                    else
                        ofsx[ox] = 1;
                    if (ity != ofsy.end())
                        ofsy[oy]++;
                    else
                        ofsy[oy] = 1;
                }
            }
        if (ofsx.size() > 1 || ofsy.size() > 1) {
            printf("Inconsistent X,Y offsets:\n");
            for (auto &&ofs : ofsx)
                printf("X [%d] = %d\n", ofs.first, ofs.second);
            for (auto &&ofs : ofsy)
                printf("Y [%d] = %d\n", ofs.first, ofs.second);
            continue;
        }
        int ox = ofsx.begin()->first;
        int oy = ofsy.begin()->first;
#if DEBUG_REINDEXING == 3
        printf("Offset r: %d, c: %d\n", oy, ox);
#endif
        // generate final indexing matrix...
        cv::Mat result(board_defs[obs.board_id].rows,
                       board_defs[obs.board_id].cols, CV_32S);
        result.setTo(-1);
        int minx = std::max(0, -ox), miny = std::max(0, -oy);
        ox += minx;
        oy += miny;
        int cols = std::min(result.cols - minx, rotated.cols - ox);
        int rows = std::min(result.rows - miny, rotated.rows - oy);
        rotated(cv::Rect(ox, oy, cols, rows))
            .copyTo(result(cv::Rect(minx, miny, cols, rows)));
#if DEBUG_REINDEXING == 3
        std::cout << obs.board << std::endl;
        std::cout << "RESULT:" << std::endl;
        std::cout << result << std::endl;
#endif
        // finally re-order corners...
        std::vector<cv::Point2f> reordered(result.size().area(),
                                           cv::Point2f(-1, -1));
        int cnt = 0;
        for (int r = 0; r < result.rows; ++r)
            for (int c = 0; c < result.cols; ++c) {
                if (result.at<int>(r, c) != -1) {
                    reordered[cnt] = obs.corner_locations[result.at<int>(r, c)];
                    result.at<int>(r, c) = cnt;
                }
                ++cnt;
            }
        obs.board = result;
        obs.indexed = true;
        obs.corner_locations = reordered;
#if DEBUG_REINDEXING == 3
        std::cout << "FINAL RESULT:" << std::endl;
        std::cout << obs.board << std::endl;
#endif
    }
    sort(boards.begin(), boards.end(), BoardObservation::orderById);
}

void transformFromBoardTriangleLocation(int rotation, int r0, int c0,
                                        int side_len, int &transr, int &transc)
{
    side_len -= 1;
    // compute expected position taking into account rotation from r0:
    switch (rotation) {
        case 1: // r0 lower -> r1 lower
        case 4:
            transr = c0;
            transc = side_len - (r0 + c0);
            // now rotation other than r0 also influences which corner of
            // triangle will be found
            transc--;
            break;
        case 2: // r0 lower -> r2 lower
        case 5:
            transr = side_len - (r0 + c0);
            transc = r0;
            // now rotation other than r0 also influences which corner of
            // triangle will be found
            transr--;
            break;
        default: // r0 lower -> r0 lower, or invalid rotation... do not do
                 // anything
            transr = r0;
            transc = c0;
            break;
    }
    // coordinate system in upper triangle is reversed
    if (rotation > 2) {
        transr = side_len - transr;
        transc = side_len - transc;
    }
}

void transformToBoardTriangleLocation(int rotation, int transr, int transc,
                                      int side_len, int &r0, int &c0)
{
    side_len -= 1;
    // coordinate system in upper triangle is reversed
    if (rotation > 2) {
        transr = side_len - transr;
        transc = side_len - transc;
    }
    // compute expected position taking into account rotation from r0:
    switch (rotation) {
        case 1: // r1 lower -> r0 lower
        case 4:
            // now rotation other than r0 also influences which corner of
            // triangle will be found transc++;
            r0 = side_len - (transr + transc);
            c0 = transr;
            break;
        case 2: // r2 lower -> r0 lower
        case 5:
            // now rotation other than r0 also influences which corner of
            // triangle will be found transr++;
            r0 = transc;
            c0 = side_len - (transr + transc);
            break;
        default: // r0 lower -> r0 lower, or invalid rotation... do not do
                 // anything
            r0 = transr;
            c0 = transc;
            break;
    }
}

bool TaggedBoardIndexer::detectDeltaTag(const cv::Mat &img,
                                        BoardObservation &obs, int r, int c,
                                        bool lower, TagDetection &det)
{
    cv::Mat &m = obs.board;
    // std::vector<TagDetection> &t = obs.tags;
    std::vector<cv::Point2f> &cor = obs.corner_locations;
    std::vector<cv::Point2f> p(3);

    // location of the tag...
    int ox, oy;
    if (lower) {
        if (m.at<int>(r, c) != -1 && m.at<int>(r, c + 1) != -1 &&
            m.at<int>(r + 1, c) != -1) {
            ox = c;
            oy = r;
            p[0] = cor[m.at<int>(r, c)];
            p[1] = cor[m.at<int>(r, c + 1)];
            p[2] = cor[m.at<int>(r + 1, c)];
        }
        else
            return false;
    }
    else {
        if (!lower && m.at<int>(r, c + 1) != -1 && m.at<int>(r + 1, c) != -1 &&
            m.at<int>(r + 1, c + 1) != -1) {
            ox = c + 1;
            oy = r + 1;
            p[0] = cor[m.at<int>(r + 1, c + 1)];
            p[1] = cor[m.at<int>(r + 1, c)];
            p[2] = cor[m.at<int>(r, c + 1)];
        }
        else
            return false;
    }

    DelTag tr(p);
    for (auto &&d : detectors) {
        det = identifyTagFromTriangle<uint8_t>(img, *(d.second), tr);
        if (det.good && det.hammingDistance < 2)
            break;
    }

    if (det.good) {
#ifdef DEBUG_REINDEXING
        printf(
            "%s: %f,%f, %f,%f, %f,%f, code: %llx, id: %d, rotation: %d, at r: "
            "%d, c: %d\n",
            lower ? "LowerT" : "UpperT", p[0].x, p[0].y, p[1].x, p[1].y, p[2].x,
            p[2].y, det.obsCode, det.id, det.rotation, oy, ox);
#endif
        // localize tag on boards...
        auto tag = tag_to_board_map.find(det.id);
        if (tag != tag_to_board_map.end()) {
            // compute real position of the tag on the board under this rotation

            const BoardDefinition &def = board_defs[tag->second.first];

            // get position on the board without rotation (r0)
            int r0 = tag->second.second.y;
            int c0 = tag->second.second.x;
            if (!lower)
                det.rotation += 3;
            int realr, realc;
            transformFromBoardTriangleLocation(det.rotation, r0, c0, def.rows,
                                               realr, realc);

#ifdef DEBUG_REINDEXING
            printf("FOUND tag: %d, at r: %d, c: %d from board: %d, fam: %s, "
                   "hamming: "
                   "%d, rotation: %d, expected at r: %d, c: %d, found as "
                   "corner: %d, "
                   "at: %d, %d\n",
                   det.id, r0, c0, tag->second.first, det.tagFamily,
                   det.hammingDistance, det.rotation, realr, realc,
                   m.at<int>(oy, ox), oy, ox);
#endif
            // repurpose some of the variables, to store board id, and position
            det.hammingDistance = tag->second.first;
            det.code = tag->second.second.y;    // row on board
            det.obsCode = tag->second.second.x; // column on board

            // vote for rotation
            rotation_histo[det.rotation]++;
            board_id_histo[tag->second.first]++;

            // compute offsets
            ox -= realc;
            oy -= realr;
            auto itx = offsetx_histo.find(ox), ity = offsety_histo.find(oy);
            if (itx != offsetx_histo.end())
                offsetx_histo[ox]++;
            else
                offsetx_histo[ox] = 1;
            if (ity != offsety_histo.end())
                offsety_histo[oy]++;
            else
                offsety_histo[oy] = 1;

#ifdef DEBUG_REINDEXING
            cv::line(dbg, cv::Point(p[0].x * 65536, p[0].y * 65536),
                     cv::Point(p[1].x * 65536, p[1].y * 65536),
                     cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 16);
            cv::line(dbg, cv::Point(p[0].x * 65536, p[0].y * 65536),
                     cv::Point(p[2].x * 65536, p[2].y * 65536),
                     cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 16);
#endif
        }
        else {
            // unknown tag... ignore it...
            det.id = -1;
        }
#ifdef DEBUG_REINDEXING
// cv::imshow("output", dbg);
// cv::waitKey(0);
#endif
        return det.good;
    }
    return false;
}

void TaggedBoardIndexer::fixTriangleBoards(
    const cv::Mat &img, std::vector<BoardObservation> &boards)
{
    for (size_t b = 0; b < boards.size(); ++b) {
        // prepare histograms
        rotation_histo.resize(6);
        std::fill(rotation_histo.begin(), rotation_histo.end(), 0);
        board_id_histo.resize(board_defs.size());
        std::fill(board_id_histo.begin(), board_id_histo.end(), 0);
        offsetx_histo.clear();
        offsety_histo.clear();

        std::vector<int> tags;
        // form Quads out of each board and
        BoardObservation &obs = boards[b];
        std::vector<std::pair<float, float>> p(3);
        obs.tags.resize(obs.corner_locations.size());
        cv::Mat &m = obs.board;
#ifdef DEBUG_REINDEXING
        std::cout << m << std::endl;
#endif
        obs.tags.clear();
        // observations are always -1 padded...
        for (int r = 0; r < obs.board.rows - 1; ++r) {
            for (int c = 0; c < obs.board.cols - 1; ++c) {
                // for each complete quad...
                TagDetection det;
                // check lower triangle
                if (detectDeltaTag(img, obs, r, c, true, det))
                    obs.tags.push_back(det);
                // check upper triangle
                if (detectDeltaTag(img, obs, r, c, false, det))
                    obs.tags.push_back(det);
            }
        }
        obs.board_id = -1;
        obs.indexed = false;

        if (obs.tags.size() == 0)
            // no tags... no worries about consistency and indexing...
            continue;

        // check histos consistency
        int best_bid = -1, mx = 0, inc = 0;
        for (size_t r = 0; r < board_id_histo.size(); ++r) {
            if (board_id_histo[r] > mx) {
                best_bid = int(r);
                if (best_bid !=
                    -1) // count inconsistent observations of board ids
                    inc += mx;
                mx = board_id_histo[r];
            }
            else
                inc = board_id_histo[r];
        }
        if (best_bid == -1 || inc > 0) {
            printf(
                "Inconsistent board ids, best: %d votes, inconsistent: %d.\n",
                mx, inc);
            continue;
        }
        int best_r = -1;
        mx = 0, inc = 0;
        for (size_t r = 0; r < rotation_histo.size(); ++r) {
            if (rotation_histo[r] > mx) {
                best_r = int(r);
                if (best_r !=
                    -1) // count inconsistent observations of rotations
                    inc += mx;
                mx = rotation_histo[r];
            }
            else
                inc = rotation_histo[r];
        }
        if (best_r == -1 || inc > 0) {
#ifdef DEBUG_REINDEXING
            printf("Inconsistent tag orientations, best: %d votes, "
                   "inconsistent: %d.\n",
                   mx, inc);
#endif
            continue;
        }

        // check consistency of offsets
        if (offsetx_histo.size() > 1 || offsety_histo.size() > 1) {
#ifdef DEBUG_REINDEXING
            printf("Inconsistent X,Y offsets:\n");
            for (auto &&ofs : offsetx_histo)
                printf("X [%d] = %d\n", ofs.first, ofs.second);
            for (auto &&ofs : offsety_histo)
                printf("Y [%d] = %d\n", ofs.first, ofs.second);
#endif
            continue;
        }

        int ox = offsetx_histo.begin()->first;
        int oy = offsety_histo.begin()->first;
        obs.board_id = best_bid;
#ifdef DEBUG_REINDEXING
        printf("Consistent observation of board %d, rotation: %d, Offset r: "
               "%d, c: "
               "%d\n",
               obs.board_id, best_r, oy, ox);
#endif
        const BoardDefinition &def = board_defs[obs.board_id];
        // transform points to match the definition board and reorder corners to
        // match
        cv::Mat result(def.rows, def.cols, CV_32S);
        result.setTo(cv::Scalar(-1));
        for (int r = 0; r < obs.board.rows; ++r) {
            for (int c = 0; c < obs.board.cols; ++c) {
                if (m.at<int>(r, c) != -1) {
                    int r0, c0;
                    transformToBoardTriangleLocation(best_r, r - oy, c - ox,
                                                     def.rows, r0, c0);
                    if (r0 < 0 || c0 < 0 || r0 >= result.rows ||
                        c0 >= result.cols)
                        continue;
                    result.at<int>(r0, c0) = m.at<int>(r, c);
                }
            }
        }
        int cnt = 0;
        std::vector<cv::Point2f> reordered(result.size().area(),
                                           cv::Point2f(-1, -1));
        for (int r = 0; r < result.rows; ++r)
            for (int c = 0; c < result.cols; ++c) {
                if (result.at<int>(r, c) != -1) {
                    reordered[cnt] = obs.corner_locations[result.at<int>(r, c)];
                    result.at<int>(r, c) = cnt;
                }
                ++cnt;
            }
        obs.board = result;
        obs.corner_locations = reordered;
        obs.indexed = true;
#ifdef DEBUG_REINDEXING
        std::cout << "FINAL RESULT:" << std::endl;
        std::cout << obs.board << std::endl;
#endif
    }
    sort(boards.begin(), boards.end(), BoardObservation::orderById);
}

} // namespace calibration
} // namespace orp
