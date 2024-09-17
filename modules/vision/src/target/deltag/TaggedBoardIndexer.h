#pragma once

#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "DetectorTools.h"

namespace orp {
namespace calibration {
typedef std::map<std::string, std::shared_ptr<TagFamily>> TagFamilies;

struct BoardDefinition
{
    typedef std::map<int, cv::Point> BoardTagMap;
    typedef BoardTagMap::iterator BoardTagMapIterator;

    int id;
    int target_id;
    std::string family;
    float border_bits;
    int cols, rows;
    double size;
    bool triangular;
    BoardTagMap tag_locations; // code -> location of lower left corner on the
                               // checkerboard
    cv::Mat corner_locations;  // matrix of Vec3d -> 3D locations of all corners
    std::shared_ptr<TagFamily> detector;
};

const Deltille::TagCodes &tagFamilyNameToCodes(const std::string &family);

void readBoardDefinitions(std::istream &in, std::vector<BoardDefinition> &defs,
                          TagFamilies &detectors);
bool updateBoardsWithCalibratedTargetFile(std::istream &in,
                                          std::vector<BoardDefinition> &defs);

void writeBoardObservations(const char *filename,
                            const std::vector<BoardDefinition> &defs,
                            const std::vector<BoardObservation> &boards);

template <typename Point3d, typename Point2d>
void get2Dto3DCorrespondences(BoardDefinition &def, BoardObservation &board,
                              std::vector<Point3d> &objectPoints,
                              std::vector<Point2d> &imagePoints)
{
    std::vector<cv::Point2f> &obs_corners = board.corner_locations;
    objectPoints.reserve(obs_corners.size());
    imagePoints.reserve(obs_corners.size());
    for (int j = 0; j < obs_corners.size(); j++) {
        if (obs_corners[j].x != -1 && obs_corners[j].y != -1) {
            int r = j / def.cols, c = j % def.cols;
            cv::Vec3d &pt = def.corner_locations.at<cv::Vec3d>(r, c);
            objectPoints.push_back(Point3d(pt[0], pt[1], pt[2]));
            imagePoints.push_back(Point2d(obs_corners[j].x, obs_corners[j].y));
        }
    }
}

int fixFullCheckerBoardOrientations(const cv::Mat &img,
                                    const cv::Size &board_size,
                                    std::vector<BoardObservation> &boards);
bool fixFullCheckerBoardOrientation(const cv::Mat &img,
                                    const cv::Size &board_size,
                                    BoardObservation &board);

struct TaggedBoardIndexer
{
    typedef std::map<int, std::pair<int, cv::Point>> TagToBoardMap;

    TagFamilies detectors;
    std::vector<BoardDefinition> board_defs;
    TagToBoardMap tag_to_board_map;
    cv::Mat dbg;
    int chessboard_row;
    int chessboard_col;

    TaggedBoardIndexer() {}

    void init()
    {
        detectors.clear();
        board_defs.clear();
        tag_to_board_map.clear();
        dbg = cv::Mat();
        chessboard_row = 13;
        chessboard_col = 13;
    }

    bool hasDefinitions() const { return board_defs.size() > 0; }

    void addBoardDefinitions(const std::string &filename)
    {
        std::ifstream f(filename);
        if (f.good())
            readBoardDefinitions(f, board_defs, detectors);
        updateBoardDefinitions();
    }

    bool readCalibratedTargetFile(const std::string &filename)
    {
        std::ifstream f(filename);
        if (!f.good()) {
            return false;
        }

        return updateBoardsWithCalibratedTargetFile(f, board_defs);
    }

    void updateBoardDefinitions()
    {
        int maxr = 0, maxc = 0;
        for (size_t i = 0; i < board_defs.size(); i++) {
            if (maxr < board_defs[i].rows)
                maxr = board_defs[i].rows;
            if (maxc < board_defs[i].cols)
                maxc = board_defs[i].cols;
        }
        chessboard_row = maxr;
        chessboard_col = maxc;
        tag_to_board_map.clear();

        for (size_t b = 0; b < board_defs.size(); ++b) {
            int board_tag_offset = board_defs[b].detector->tagFamilyOffset;
            for (auto &&tl : board_defs[b].tag_locations)
                tag_to_board_map[tl.first + board_tag_offset] =
                    std::make_pair(int(b), tl.second);
        }
    }

    void setDebugImage(cv::Mat &dbg_image) { dbg = dbg_image; }

    void fixCheckerBoards(const cv::Mat &img,
                          std::vector<BoardObservation> &boards);
    void fixTriangleBoards(const cv::Mat &img,
                           std::vector<BoardObservation> &boards);
    bool detectDeltaTag(const cv::Mat &img, BoardObservation &obs, int r, int c,
                        bool lower, TagDetection &det);

private:
    std::vector<int> rotation_histo;
    std::vector<int> board_id_histo;
    std::map<int, int> offsetx_histo;
    std::map<int, int> offsety_histo;
};

}; // namespace calibration
}; // namespace orp
