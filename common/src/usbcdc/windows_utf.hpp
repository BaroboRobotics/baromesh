#ifndef DONGLEDEVICEPATH_WINDOWS_UTF_HPP
#define DONGLEDEVICEPATH_WINDOWS_UTF_HPP

#include <exception>
#include <string>
#include <vector>

#include <windows.h>

// Credit: Stackoverflow rubenvb, but fixed up by Harris

// UTF16 -> UTF8 conversion
static inline std::string toUtf8 (const std::wstring& input) {
    // Compute size of needed buffer in bytes
    auto size = WideCharToMultiByte(CP_UTF8, 0,        // codepage, flags
                                    input.data(), -1,  // input
                                    nullptr, 0,        // output
                                    nullptr, nullptr   // unused for UTF8
                                    );
    if (size) {
        auto result = std::vector<char>(size);
        if (WideCharToMultiByte(CP_UTF8, 0,
                                input.data(), -1,
                                result.data(), result.size(),
                                nullptr, nullptr
                                ) > 0) {
            return result.data();
        }
        else {
            throw std::runtime_error("Failure to execute toUtf8: conversion failed." );
        }
    }
    return {};
}

// UTF8 -> UTF16 conversion
static inline std::wstring toUtf16 (const std::string &input) {
    // Compute size of needed buffer in wide characters
    auto size = MultiByteToWideChar(CP_UTF8, 0,
                                    input.data(), -1,
                                    nullptr, 0
                                    );
    if (size) {
        auto result = std::vector<wchar_t>(size);
        if (MultiByteToWideChar(CP_UTF8, 0,
                                input.data(), -1,
                                result.data(), result.size()
                                ) > 0) {
            return result.data();
        }
        else {
            throw std::runtime_error("Failure to execute toUtf16: conversion failed.");
        }
    }
    return {};
}

#endif