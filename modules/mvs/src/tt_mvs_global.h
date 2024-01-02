#pragma once

#include <AxCore/platform.h>

namespace thoht {

#ifndef AX_STATIC
    #if defined(AX_MVS_EXPORT)
        #define AX_MVS_API AX_DECL_EXPORT
    #else
        #define AX_MVS_API AX_DECL_IMPORT
    #endif    
#else
    #define AX_MVS_API
#endif

}