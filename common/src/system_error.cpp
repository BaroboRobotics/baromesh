#include <baromesh/system_error.hpp>

namespace baromesh {

const char* ErrorCategory::name () const BOOST_NOEXCEPT {
    return "baromesh";
}

std::string ErrorCategory::message (int ev) const BOOST_NOEXCEPT {
    switch (Status(ev)) {
        case Status::OK:
            return "OK";
        case Status::CANNOT_OPEN_DONGLE:
            return "Dongle detected, but we are unable to open the device. Try rebooting the dongle.";
        case Status::DONGLE_NOT_FOUND:
            return "No dongle was detected attached to the computer.";
        case Status::PORT_OUT_OF_RANGE:
            return "The daemon returned an invalid TCP/IP port.";
        case Status::UNREGISTERED_SERIALID:
            return "Attempt to receive message from a robot serial ID unregistered in the daemon.";
        case Status::INVALID_SERIALID:
            return "Cannot send message to an invalid robot serial ID.";
        case Status::DAEMON_UNAVAILABLE:
            return "Cannot connect to the daemon.";
        case Status::STRANGE_DONGLE:
            return "A dongle is plugged in, but we are unable to communicate with it. "
                   "Try rebooting it and/or updating its firmware.";
        case Status::INCOMPATIBLE_FIRMWARE:
            return "A dongle is plugged in, but it is running an incompatible firmware version. "
                   "Update its firmware.";
        case Status::BUFFER_OVERFLOW:
            return "Daemon buffer overflow.";
        case Status::OTHER_ERROR:
            return "An unspecified error occurred inside the daemon.";
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
