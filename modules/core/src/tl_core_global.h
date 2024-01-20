#pragma once

#include "util_compile.h"

namespace tl {

#ifndef AX_STATIC
#if defined(AX_CORE_EXPORT)
#define AX_CORE_API AX_DECL_EXPORT
#else
#define AX_CORE_API AX_DECL_IMPORT
#endif
#else
#define AX_CORE_API
#endif

} // namespace tl
