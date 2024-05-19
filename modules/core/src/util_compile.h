#pragma once

#ifdef _WIN32
#define TL_OS_WIN

#ifdef _WIN64
#define TL_OS_WIN64
#else
#define TL_OS_WIN32
#endif
#elif __linux__
#define TL_OS_LINUX
#elif __unix__
#define TL_OS_UNIX
#elif defined(_POSIX_VERSION)
#define TL_OS_POSIX
#else
#error "Unknown compiler"
#endif

#ifdef TL_OS_WIN
#define TL_DECL_EXPORT __declspec(dllexport)
#define TL_DECL_IMPORT __declspec(dllimport)
#else
#define TL_DECL_EXPORT __attribute__((visibility("default")))
#define TL_DECL_IMPORT __attribute__((visibility("default")))
#define TL_DECL_HIDDEN __attribute__((visibility("hidden")))
#endif
