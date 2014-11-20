#include "baromesh/system_error.hpp"

namespace baromesh {

const char* ErrorCategory::name () const noexcept {
    return "baromesh";
}

std::string ErrorCategory::message (int ev) const noexcept {
    switch (Status(ev)) {
#define ITEM(x) case Status::x: return #x;
        ITEM(OK)
        ITEM(DONGLE_NOT_FOUND)
        ITEM(PORT_OUT_OF_RANGE)
        ITEM(NO_ROBOT_ENDPOINT)
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