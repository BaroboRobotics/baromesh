#ifndef BAROMESH_DAEMON_DONGLE_HPP
#define BAROMESH_DAEMON_DONGLE_HPP

#include "basicdongle.hpp"

#include "rpc/asio/client.hpp"
#include "sfp/asio/messagequeue.hpp"

#include <boost/asio/serial_port.hpp>

namespace baromesh {

using SerialMessageQueue = sfp::asio::MessageQueue<boost::asio::serial_port>;
using SerialClient = rpc::asio::Client<SerialMessageQueue>;
using Dongle = baromesh::BasicDongle<SerialClient>;
using ZigbeeClient = rpc::asio::Client<Dongle::MessageQueue>;

} // namespace baromesh

#endif