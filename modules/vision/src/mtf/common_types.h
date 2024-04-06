#pragma once

#include <map>
#include <vector>

#include <opencv2/core/types.hpp>

using Pointlist = std::vector<cv::Point2d>;
using Boundarylist = std::map<int, Pointlist>;

#ifdef _WIN32
#define EXE_SUFFIX ".exe"
#define _WIN32_WINNT 0x0502
#else
#define EXE_SUFFIX ""
#endif

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

#ifndef uint16_t
using uint16_t = unsigned short int;
#endif

#ifndef uint32_t
using uint32_t = unsigned int;
#endif

#ifndef uint32_t
using int32_t = int;
#endif

#if _MSC_VER == 1600
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef int int32_t;

#define M_PI 3.14159265358979

__inline long int lrint(double flt)
{
    int intgr;

    _asm {
			fld flt
			fistp intgr
    }

    return intgr;
}

#endif

#define SQR(x) ((x) * (x))

// stat functions
#include <sys/stat.h>
#ifndef _WIN32
#define STAT stat
#else
#define STAT _stat
#endif

#ifdef _WIN32
#ifndef S_ISDIR
#define S_ISDIR(mode) (((mode) & S_IFMT) == S_IFDIR)
#endif
#define strncasecmp _strnicmp
#define strcasecmp _stricmp
#endif

template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}
