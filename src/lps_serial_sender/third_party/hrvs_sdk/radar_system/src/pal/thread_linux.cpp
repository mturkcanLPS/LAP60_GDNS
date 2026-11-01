#include "pal/thread_linux.h"
#include <unistd.h>

void ThreadLib::sleep(long long ms)
{
    // Convert milliseconds to microseconds
    long long microseconds = ms * 1000;
    
    // Sleep in chunks of 1,000,000 microseconds if necessary
    while (microseconds >= 1000000) {
        usleep(1000000);
        microseconds -= 1000000;
    }
    
    // Sleep for the remaining microseconds
    if (microseconds > 0) {
        usleep(microseconds);
    }
}