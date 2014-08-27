#ifndef DONGLE_EXCEPTION
#define DONGLE_EXCEPTION

#include <exception>

namespace dongle {

struct Exception : std::exception { };

struct ThreadException : Exception {
    ThreadException (std::exception_ptr eptr)
            : underlyingExceptionPtr(eptr) { }

    virtual const char* what () const noexcept override {
        return "Unable to start reader thread";
    }

    std::exception_ptr underlyingExceptionPtr;
};

struct ReadPumpException : Exception {
    ReadPumpException (std::exception_ptr eptr)
            : underlyingExceptionPtr(eptr) { }

    virtual const char* what () const noexcept override {
        return "Read error";
    }

    std::exception_ptr underlyingExceptionPtr;
};

struct DongleNotFoundException : Exception {
    virtual const char* what () const noexcept override {
        return "dongle not found";
    }
};

struct SfpConnectionException : Exception {
    virtual const char* what () const noexcept override {
        return "SFP connection failed";
    }
};

}

#endif
