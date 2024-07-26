#pragma once

#ifdef WIN32
#include <windows.h>
#endif

#ifdef UNIX
#include <time.h>
#include <sys/time.h>
#endif

class HiresTimer
{
private:
    bool started;
    bool stopped;
    double elapsed_time;
#ifdef WIN32
    LARGE_INTEGER frequency;
    LARGE_INTEGER start_time;
#endif
#ifdef UNIX
    double start_time;
#endif

public:
    HiresTimer(void);
    ~HiresTimer(void);
    void start();
    double elapsed();
    void stop();
    void reset();
};
