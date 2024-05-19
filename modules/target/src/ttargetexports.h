#pragma once

#include <tCore/platform.h>

#ifndef TL_STATIC
#if defined(TL_TARGET_EXPORT)
#define TL_TARGET_API TL_DECL_EXPORT
#else
#define TL_TARGET_API TL_DECL_IMPORT
#endif
#else
#define TL_TARGET_API
#endif
