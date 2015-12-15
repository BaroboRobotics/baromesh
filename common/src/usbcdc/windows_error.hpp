#ifndef DONGLEDEVICEPATH_WINDOWS_ERROR_HPP
#define DONGLEDEVICEPATH_WINDOWS_ERROR_HPP

#ifndef _WIN32
#error win32_error.h is a Windows-specific header file.
#endif

#include <windows.h>
//#include <tchar.h>

#include <exception>
#include <string>
#include <memory>

static inline std::string windowsErrorString (DWORD code) {
    char* errorText = nullptr;
    auto nWritten = FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM
            | FORMAT_MESSAGE_ALLOCATE_BUFFER
            | FORMAT_MESSAGE_IGNORE_INSERTS,
            nullptr, code,
            MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
            LPSTR(&errorText),
            0, nullptr);
    if (errorText && nWritten) {
        std::shared_ptr<void> guard { nullptr, [&] (void*) {LocalFree(errorText);}};
        return std::string(errorText, errorText + nWritten);
    }
    else {
        return std::string("Windows error ") + std::to_string(GetLastError())
            + " while formatting message for error " + std::to_string(code);
    }
}

struct WindowsError : std::runtime_error {
    WindowsError (std::string prefix, DWORD code)
        : std::runtime_error{prefix + ": " + windowsErrorString(code)}
    {}
};

#endif
