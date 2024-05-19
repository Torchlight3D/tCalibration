#pragma once

#include "platform.h"

#ifndef TL_STATIC
#if defined(TL_CORE_EXPORT)
#define TL_CORE_API TL_DECL_EXPORT
#else
#define TL_CORE_API TL_DECL_IMPORT
#endif
#else
#define TL_CORE_API
#endif
