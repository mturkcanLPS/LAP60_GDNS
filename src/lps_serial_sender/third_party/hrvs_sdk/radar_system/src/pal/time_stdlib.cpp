#include "pal/time_stdlib.h"
#include <chrono>

long long TimeStdlib::getTime()
{
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    return now_ms.time_since_epoch().count();
}
