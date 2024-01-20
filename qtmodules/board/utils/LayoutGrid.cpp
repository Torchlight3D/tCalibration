#include "LayoutGrid.h"

namespace tl {

LayoutGrid::LayoutGrid() : mNeedUpdate(false) {}

void LayoutGrid::updateLayout()
{
    if (mNeedUpdate || (mRect != mPreviousRect)) {
        QCPLayoutGrid::updateLayout();
        mPreviousRect = mRect;
        mNeedUpdate = false;
    }
}

void LayoutGrid::needUpdate(bool needUpdate) { mNeedUpdate = needUpdate; }

bool LayoutGrid::needUpdate() const { return mNeedUpdate; }

} // namespace tl
