#ifndef DONGLE_EXCEPTION
#define DONGLE_EXCEPTION

#include <exception>

namespace dongle {

// Exceptions that can be thrown from methods of class Transport
struct Exception : std::exception {};

struct NotConnected : Exception {
    virtual const char* what () const noexcept override {
        return "dongle not connected during write";
    }
};

struct WrappedException : Exception {
    WrappedException(std::exception_ptr eptr, std::string msg)
            : mWhat(wrapWhat(eptr, msg)) { }

    virtual const char* what () const noexcept override final {
        return mWhat.c_str();
    }

private:

    std::string mWhat;
    std::string wrapWhat(std::exception_ptr eptr, std::string what) {
        try {
            std::rethrow_exception(eptr);
        }
        catch (std::exception &e) {
            return what + std::string(e.what());
        }
        catch (...) {
            return "(wat?)";
        }
        return "";
    }
};

struct SerialError : WrappedException {
    SerialError (std::exception_ptr ep)
            : WrappedException(ep, "serial error during write: ") { }
};

struct ThreadError : WrappedException {
    ThreadError (std::exception_ptr eptr)
            : WrappedException(eptr, "thread creation error: ") { }
};

// The following are PRIVATE (not thrown to client code)

struct DongleNotFoundException : Exception {
    virtual const char* what () const noexcept override {
        return "dongle not found";
    }
};

} // namespace dongle

#endif
