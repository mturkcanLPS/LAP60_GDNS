#include "pal/time_linux.h"
#include <sys/time.h>
#include <cstddef>

long long TimeLib::getTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long milliseconds = (long long)(tv.tv_sec) * 1000 + (long long)(tv.tv_usec) / 1000;
    return milliseconds;
}
