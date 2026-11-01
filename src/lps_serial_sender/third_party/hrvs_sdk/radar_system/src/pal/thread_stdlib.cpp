#include "pal/thread_stdlib.h"
#include <chrono>
#include <thread>

void ThreadStdlib::sleep(long long ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
