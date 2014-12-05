#ifndef BAROMESH_DAEMON_DONGLEDEVICEPATH_HPP
#define BAROMESH_DAEMON_DONGLEDEVICEPATH_HPP

#include <boost/system/error_code.hpp>

#include <string>

namespace baromesh {

std::string dongleDevicePath (boost::system::error_code& ec);

} // namespace baromesh

#endif