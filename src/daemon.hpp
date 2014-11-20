#ifndef BAROMESH_DAEMON_HPP
#define BAROMESH_DAEMON_HPP

#include "basicdaemon.hpp"

#include "rpc/asio/client.hpp"
#include "sfp/asio/messagequeue.hpp"

#include <boost/asio.hpp>

namespace baromesh {

using TcpMessageQueue = sfp::asio::MessageQueue<boost::asio::ip::tcp::socket>;
using TcpClient = rpc::asio::Client<TcpMessageQueue>;
using Daemon = BasicDaemon<TcpClient>;

} // namespace baromesh

#endif