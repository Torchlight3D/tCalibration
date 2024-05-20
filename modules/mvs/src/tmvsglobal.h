#pragma once

#include <tCore/platform.h>

namespace tl {

#ifndef TL_STATIC
#if defined(TL_MVS_EXPORT)
#define TL_MVS_API TL_DECL_EXPORT
#else
#define TL_MVS_API TL_DECL_IMPORT
#endif
#else
#define TL_MVS_API
#endif

} // namespace tl