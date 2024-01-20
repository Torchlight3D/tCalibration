#include "StereoModuleData.h"

namespace tl {

bool StereoImageData::isValid() const
{
    return !left.empty() && !right.empty() && timestamp > 0.;
}

} // namespace tl
