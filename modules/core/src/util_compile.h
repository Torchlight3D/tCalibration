#pragma once

#ifdef _WIN32
#define AX_OS_WIN

#ifdef _WIN64
#define AX_OS_WIN64
#else
#define AX_OS_WIN32
#endif
#elif __linux__
#define AX_OS_LINUX
#elif __unix__
#define AX_OS_UNIX
#elif defined(_POSIX_VERSION)
#define AX_OS_POSIX
#else
#error "Unknown compiler"
#endif

#ifdef AX_OS_WIN
#define AX_DECL_EXPORT __declspec(dllexport)
#define AX_DECL_IMPORT __declspec(dllimport)
#else
#define AX_DECL_EXPORT __attribute__((visibility("default")))
#define AX_DECL_IMPORT __attribute__((visibility("default")))
#define AX_DECL_HIDDEN __attribute__((visibility("hidden")))
#endif
