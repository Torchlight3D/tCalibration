#pragma once

#include <stdio.h>
#include <string>

class Logger
{
public:
    enum log_level_t
    {
        LOGGER_NONE = 0,
        LOGGER_INFO = 1,
        LOGGER_DEBUG = 2,
        LOGGER_ERROR = 4
    };
    Logger() : destination(stdout) {}

    void redirect(const std::string &fname, bool append = false)
    {
        destination = fopen(fname.c_str(), append ? "a" : "w");
        if (!destination) {
            fprintf(stderr, "Could not create %s for writing\n", fname.c_str());
            destination = stdout;
        }
    }

    ~Logger()
    {
        if (destination != stdout) {
            fclose(destination);
        }
    }

    void flush() { fflush(destination); }

    void enable_level(log_level_t ll) { log_level |= ll; }

    void disable_level(log_level_t ll) { log_level &= ~ll; }

    template <class... T>
    void info(T... t)
    {
        if (log_level & LOGGER_INFO) {
            fprintf(destination, t...);
        }
    }

    template <class... T>
    void debug(T... t)
    {
        if (log_level & LOGGER_DEBUG) {
            fprintf(destination, t...);
        }
    }

    template <class... T>
    void error(T... t)
    {
        if (log_level & LOGGER_ERROR) {
            fprintf(destination, t...);
            fflush(destination);
        }
    }

private:
    FILE *destination;
    int log_level = LOGGER_INFO | LOGGER_ERROR;
};

extern Logger logger;
