#include "rundongle.hpp"

#ifdef _WIN32
#include "daemonservice/win32.cpp"
#elif defined(__linux__)
#include "daemonservice/linux.cpp"
#elif defined(__APPLE__) && defined(__MACH__)
#include "daemonservice/osx.cpp"
#else
#error No daemonservice.cpp available for this platform.
#endif
