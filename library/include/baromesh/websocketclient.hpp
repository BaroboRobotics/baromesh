#ifndef BAROMESH_WEBSOCKETCLIENT_HPP
#define BAROMESH_WEBSOCKETLIENT_HPP

#include "rpc/asio/client.hpp"

#include <baromesh/websocketmessagequeue.hpp>

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include <utility>

namespace baromesh {

using WebSocketClient
    = rpc::asio::Client<websocket::MessageQueue<websocketpp::config::asio_client>>;

} // namespace baromesh

#endif
