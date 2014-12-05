#ifndef BAROMESH_STATUS_HPP
#define BAROMESH_STATUS_HPP

namespace baromesh {

enum class Status {
    OK,
    DONGLE_NOT_FOUND,
    PORT_OUT_OF_RANGE,
    NO_ROBOT_ENDPOINT,

    UNREGISTERED_SERIALID
};

} // namespace baromesh

#endif