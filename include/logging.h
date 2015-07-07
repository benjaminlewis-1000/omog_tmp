// Logging class, C++

#ifndef LOGGING_H
#define LOGGING_H

#include <sstream>
#include <boost/format.hpp>
#include <iostream>
#include <fstream> 
#include <string>

enum log_level_t {
    LOG_NOTHING,
    LOG_CRITICAL,
    LOG_ERROR,
    LOG_WARNING,
    LOG_IMPORTANT_INFO,
    LOG_INFO,
    LOG_DEBUG
};

#define GLOBAL_LEVEL LOG_IMPORTANT_INFO

class mystreambuf: public std::streambuf
{
};
mystreambuf nostreambuf;
std::ostream nocout(&nostreambuf);
#define log(x) ((x <= GLOBAL_LEVEL)? std::cout : nocout)

#endif
