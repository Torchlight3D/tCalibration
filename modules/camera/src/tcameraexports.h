#pragma once

#ifndef TL_STATIC
#if defined(TL_CAMERA_EXPORT)
#define TL_CAMERA_API TL_DECL_EXPORT
#else
#define TL_CAMERA_API TL_DECL_IMPORT
#endif
#else
#define TL_CAMERA_API
#endif