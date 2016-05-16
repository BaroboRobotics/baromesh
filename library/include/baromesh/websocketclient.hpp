#ifndef BAROMESH_WEBSOCKETCLIENT_HPP
#define BAROMESH_WEBSOCKETCLIENT_HPP

#include <rpc/asio/client.hpp>

#include <baromesh/websocketconnector.hpp>

#include <utility>

namespace baromesh {

using WebSocketClient = rpc::asio::Client<websocket::Connector::MessageQueue>;

} // namespace baromesh

#endif
