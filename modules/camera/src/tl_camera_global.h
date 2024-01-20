#pragma once

#include <tCore/platform.h>

namespace tl {

#ifndef AX_STATIC
    #if defined(AX_CAMERA_EXPORT)
        #define AX_CAMERA_API AX_DECL_EXPORT
    #else
        #define AX_CAMERA_API AX_DECL_IMPORT
    #endif    
#else
    #define AX_CAMERA_API
#endif

}