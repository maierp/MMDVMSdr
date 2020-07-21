#include <sys/time.h>
#include <iostream>

#include "Debug.h"

//void LOGCONSOLE(int x, int y, const char* msg) {
//    struct timeval now;
//    gettimeofday(&now, NULL);
//    struct tm* tm = gmtime(&now.tv_sec);
//    char buffer[30];
//    sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d.%03lu ", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec, now.tv_usec / 1000U);
//    std::cout << buffer << msg << std::endl;
//}