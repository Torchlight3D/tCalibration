#include "StereoModuleData.h"

namespace thoht {

bool StereoImageData::isValid() const
{
    return !left.empty() && !right.empty() && timestamp > 0.;
}

} // namespace thoht
