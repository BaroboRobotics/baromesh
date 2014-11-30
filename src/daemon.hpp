#ifndef BAROMESH_DAEMON_HPP
#define BAROMESH_DAEMON_HPP

#include "basicdaemon.hpp"

#include "rpc/asio/client.hpp"
#include "sfp/asio/messagequeue.hpp"

#include <boost/asio.hpp>

#include <boost/log/common.hpp>
#include <boost/log/sources/logger.hpp>

#include <memory>

namespace baromesh {

using TcpMessageQueue = sfp::asio::MessageQueue<boost::asio::ip::tcp::socket>;
using TcpClient = rpc::asio::Client<TcpMessageQueue>;
using Daemon = BasicDaemon<TcpClient>;

using AcquireDaemonHandlerSignature = void(boost::system::error_code, std::shared_ptr<Daemon>);
using AcquireDaemonHandler = std::function<AcquireDaemonHandlerSignature>;

void asyncAcquireDaemonImpl (AcquireDaemonHandler handler);

// Instantiate a daemon singleton pointer, connect it to the daemon process,
// then return the pointer to the caller via the supplied handler.
template <class Handler>
BOOST_ASIO_INITFN_RESULT_TYPE(Handler,
    AcquireDaemonHandlerSignature)
asyncAcquireDaemon (Handler&& handler) {
    boost::asio::detail::async_result_init<
        Handler, AcquireDaemonHandlerSignature
    > init { std::forward<Handler>(handler) };
    auto& realHandler = init.handler;

    asyncAcquireDaemonImpl(realHandler);

    return init.result.get();
}

} // namespace baromesh

#endif