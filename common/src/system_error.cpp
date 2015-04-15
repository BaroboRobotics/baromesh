#include "baromesh/system_error.hpp"

namespace baromesh {

const char* ErrorCategory::name () const BOOST_NOEXCEPT {
    return "baromesh";
}

std::string ErrorCategory::message (int ev) const BOOST_NOEXCEPT {
    switch (Status(ev)) {
#define ITEM(x) case Status::x: return #x;
        ITEM(OK)
        ITEM(CANNOT_OPEN_DONGLE)
        ITEM(DONGLE_NOT_FOUND)
        ITEM(PORT_OUT_OF_RANGE)
        ITEM(NO_ROBOT_ENDPOINT)

        ITEM(UNREGISTERED_SERIALID)
        ITEM(INVALID_SERIALID)
        ITEM(DAEMON_UNAVAILABLE)

        ITEM(STRANGE_DONGLE)
        ITEM(DONGLE_VERSION_MISMATCH)

        ITEM(BUFFER_OVERFLOW)
        ITEM(OTHER_ERROR)
#undef ITEM
        default: return "(unknown status)";
    }
}

const boost::system::error_category& errorCategory () {
    static ErrorCategory instance;
    return instance;
}

boost::system::error_code make_error_code (Status status) {
    return boost::system::error_code(static_cast<int>(status),
        errorCategory());
}

boost::system::error_condition make_error_condition (Status status) {
    return boost::system::error_condition(static_cast<int>(status),
        errorCategory());
}

} // namespace baromesh