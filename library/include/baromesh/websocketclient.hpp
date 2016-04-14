#ifndef BAROMESH_WEBSOCKETCLIENT_HPP
#define BAROMESH_WEBSOCKETLIENT_HPP

#include "rpc/asio/client.hpp"

#include <baromesh/websocketmessagequeue.hpp>

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include <utility>

namespace baromesh {

using WebSocketClient = rpc::asio::Client<websocket::MessageQueue<websocketpp::config::asio_client>>;

typedef void InitWebSocketClientHandlerSignature(boost::system::error_code);

#if 0
template <class Handler>
BOOST_ASIO_INITFN_RESULT_TYPE(Handler, InitWebSocketClientHandlerSignature)
asyncInitWebSocketClient (websocketpp::client<websocket::config::asio_client>& connector,
        WebSocketClient& client, const websocketpp::uri& uri, Handler&& handler) {
    boost::asio::detail::async_result_init<
        Handler, InitWebSocketClientHandlerSignature
    > init { std::forward<Handler>(handler) };


    boost::asio::async_connect(client.messageQueue().stream(), iter,
        [&client, realHandler] (boost::system::error_code ec, Tcp::resolver::iterator iter) {
            if (!ec) {
                auto log = client.log();
                BOOST_LOG(log) << "WebSocket client connected to " << iter->endpoint();
                client.messageQueue().asyncHandshake([&client, realHandler] (boost::system::error_code ec) {
                    client.get_io_service().post(std::bind(realHandler, ec));
                });
            }
            else {
                client.get_io_service().post(std::bind(realHandler, ec));
            }
        });

    return init.result.get();
}
#endif
} // namespace baromesh

#endif
