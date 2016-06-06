#ifndef BAROMESH_DONGLEDEVICEPATH_HPP
#define BAROMESH_DONGLEDEVICEPATH_HPP

#include <boost/system/error_code.hpp>

#include <string>

namespace baromesh {

std::string dongleDevicePath ();
std::string dongleDevicePath (boost::system::error_code& ec);

} // namespace baromesh

#endif