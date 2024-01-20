#include "RoundedFocusFrame.h"

namespace tl {

const CornerRadius& RoundedFocusFrame::radiuses() const { return _radiuses; }

void RoundedFocusFrame::setRadiuses(const CornerRadius& radiuses)
{
    if (radiuses != _radiuses) {
        _radiuses = radiuses;
        emit radiusesChanged();
        update();
    }
}

} // namespace tl

#include "moc_RoundedFocusFrame.cpp"
