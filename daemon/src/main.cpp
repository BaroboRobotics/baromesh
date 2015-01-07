#include "rundaemon.hpp"

#ifdef _WIN32
#include "main/win32.cpp"
#elif defined(__linux__)
#include "main/linux.cpp"
#elif defined(__APPLE__) && defined(__MACH__)
#include "main/osx.cpp"
#else
#error No main() implemented for this platform.
#endif
