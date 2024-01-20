#pragma once

#include <tCore/platform.h>

namespace tl {

#ifndef AX_STATIC
    #if defined(AX_MATH_EXPORT)
        #define AX_MATH_API AX_DECL_EXPORT
    #else
        #define AX_MATH_API AX_DECL_IMPORT
    #endif    
#else
    #define AX_MATH_API
#endif

}