#ifndef BAROMESH_STATUS_HPP
#define BAROMESH_STATUS_HPP

#include "daemon.pb.h"

namespace baromesh {

enum class Status {
    OK                      = barobo_Daemon_Status_OK,
    CANNOT_OPEN_DONGLE      = barobo_Daemon_Status_CANNOT_OPEN_DONGLE,
    DONGLE_NOT_FOUND        = barobo_Daemon_Status_DONGLE_NOT_FOUND,
    PORT_OUT_OF_RANGE       = barobo_Daemon_Status_PORT_OUT_OF_RANGE,
    NO_ROBOT_ENDPOINT       = barobo_Daemon_Status_NO_ROBOT_ENDPOINT,

    UNREGISTERED_SERIALID   = barobo_Daemon_Status_UNREGISTERED_SERIALID,
    INVALID_SERIALID        = barobo_Daemon_Status_INVALID_SERIALID,

    DAEMON_UNAVAILABLE      = barobo_Daemon_Status_DAEMON_UNAVAILABLE
};

} // namespace baromesh

#endif