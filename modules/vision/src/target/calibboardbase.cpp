#include "calibboardbase.h"

#include <glog/logging.h>

#include <nlohmann/json.hpp>

#include "chessboard.h"
#include "circlegridboard.h"
#include "kalibrapriltagboard.h"

namespace tl {

CalibBoardBase::CalibBoardBase(int rows, int cols) : m_rows(rows), m_cols(cols)
{
    m_boardPoints.reserve(cornerCount());
    m_ids.resize(cornerCount(), 0);
}

namespace key {
constexpr char kType[]{"type"};
constexpr char kConfigs[]{"configs"};
constexpr char kOptions[]{"opts"};
constexpr char kDetectOptions[]{"detect_opts"};
} // namespace key

CalibBoardBase::Ptr CalibBoardBase::fromJson(const std::string &json)
{
    const auto j = nlohmann::json::parse(json);

    // TODO: Make each derived type takes care of its own creator.
    const auto type = j.at(key::kType).get<std::string>();
    const auto j_configs = j.at(key::kConfigs);
    const auto j_opts = j_configs.at(key::kOptions);
    const auto j_detectOpts = j_configs.at(key::kDetectOptions);
    if (type == Chessboard::kType) {
        Chessboard::Options opts;
        j_opts.get_to(opts);
        Chessboard::DetectOptions detectOpts;
        j_detectOpts.get_to(detectOpts);
        return std::make_shared<Chessboard>(opts, detectOpts);
    }
    if (type == CircleGridBoard::kType) {
        CircleGridBoard::Options opts;
        j_opts.get_to(opts);
        return std::make_shared<CircleGridBoard>(opts);
    }
    if (type == KalibrAprilTagBoard::kType) {
        KalibrAprilTagBoard::Options opts;
        j_opts.get_to(opts);
        KalibrAprilTagBoard::DetectOptions detectOpts;
        j_detectOpts.get_to(detectOpts);
        return std::make_shared<KalibrAprilTagBoard>(opts, detectOpts);
    }

    LOG(WARNING) << "Unsupported calibration target type: " << type;
    return nullptr;
}

int CalibBoardBase::rows() const { return m_rows; };

int CalibBoardBase::cols() const { return m_cols; };

cv::Point3d CalibBoardBase::boardPoint(int i) const
{
    // TODO: check bound
    return m_boardPoints[i];
}

const std::vector<cv::Point3d> &CalibBoardBase::boardPoints() const
{
    return m_boardPoints;
}

const std::vector<CornerId> &CalibBoardBase::cornerIds() const { return m_ids; }

cv::Point CalibBoardBase::posOf(int i) const
{
    return {i % cols(), i / cols()};
}

int CalibBoardBase::indexOf(int r, int c) const { return cols() * r + c; }

} // namespace tl
