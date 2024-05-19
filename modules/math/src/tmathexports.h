#pragma once

#include <tCore/platform.h>

#ifndef TL_STATIC
#if defined(TL_MATH_EXPORT)
#define TL_MATH_API TL_DECL_EXPORT
#else
#define TL_MATH_API TL_DECL_IMPORT
#endif
#else
#define TL_MATH_API
#endif
