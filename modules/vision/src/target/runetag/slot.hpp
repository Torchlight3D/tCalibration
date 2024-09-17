#pragma once

#include "ellipsepoint.hpp"

namespace tl {
namespace runetag {

class Slot
{
    friend class SlotFitter;

private:
    cv::Matx33d Qmin;
    cv::Matx33d Qmax;
    bool _value;
    bool _discarded;
    EllipsePoint* payload;

public:
    cv::Point2f v1;
    cv::Point2f v2;
    cv::Point2f c;
    cv::Point2f slot_center;

    Slot();

    Slot(cv::Matx33d _Qmin, cv::Matx33d _Qmax, const cv::Point2d& _c,
         const cv::Point2d& _v1, const cv::Point2d& _v2);

    inline void invalidate()
    {
        _value = false;
        _discarded = true; /*payload = 0;*/
    }

    inline bool value() const { return _value; }

    inline bool discarded() const { return _discarded; }

    inline EllipsePoint* getPayload() const { return payload; }

    inline cv::Point2d getCenter() const { return c; }

    bool checkInside(const cv::Point2d& p) const;

    bool setIfInside(const cv::Point2d& p, EllipsePoint* _payload);
};

} // namespace runetag
} // namespace tl
