#include "calib_board_base.h"

namespace tl {

CalibBoardBase::CalibBoardBase(int rows, int cols) : m_rows(rows), m_cols(cols)
{
    m_boardPoints.reserve(cornerCount());
    m_ids.resize(cornerCount(), 0);
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
