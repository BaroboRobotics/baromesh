#ifndef BAROMESH_BASIC_HPP
#define BAROMESH_BASIC_HPP

#include "tcpclient.hpp"
#include "gen-daemon.pb.hpp"

#include "baromesh/system_error.hpp"

#include <boost/asio/async_result.hpp>
#include <boost/asio/io_service.hpp>

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <utility>

namespace baromesh {

using ResolveSerialIdHandlerSignature = void(boost::system::error_code, std::pair<std::string, std::string>);
using ResolveSerialIdHandler = std::function<ResolveSerialIdHandlerSignature>;

template <class Duration, class Handler>
BOOST_ASIO_INITFN_RESULT_TYPE(Handler, ResolveSerialIdHandlerSignature)
asyncResolveSerialId (TcpClient& daemon, std::string serialId, Duration&& timeout, Handler&& handler) {
    boost::asio::detail::async_result_init<
        Handler, ResolveSerialIdHandlerSignature
    > init { std::forward<Handler>(handler) };
    auto& realHandler = init.handler;

    assert(4 == serialId.size());
    rpc::MethodIn<barobo::Daemon>::resolveSerialId args = decltype(args)();

    strncpy(args.serialId.value, serialId.data(), 4);
    args.serialId.value[4] = 0;

    asyncFire(daemon, args, std::forward<Duration>(timeout),
        [&daemon, realHandler] (boost::system::error_code ec, rpc::MethodResult<barobo::Daemon>::resolveSerialId result) {
            auto log = daemon.log();
            try {
                if (ec) {
                    BOOST_LOG(log) << "resolveSerialId reported error: " << ec.message();
                    throw boost::system::system_error(ec);
                }

                if (!result.has_endpoint) {
                    BOOST_LOG(log) << "resolveSerialId result has no endpoint";
                    throw boost::system::system_error(Status::NO_ROBOT_ENDPOINT);
                }

                auto port = uint16_t(result.endpoint.port);
                if (port != result.endpoint.port) {
                    throw boost::system::system_error(Status::PORT_OUT_OF_RANGE);
                }

                using std::to_string;
                daemon.get_io_service().post(
                    std::bind(realHandler, boost::system::error_code(),
                        std::make_pair(std::string(result.endpoint.address), to_string(port))));
            }
            catch (boost::system::system_error& e) {
                BOOST_LOG(log) << "resolveSerialId: " << e.what();
                daemon.get_io_service().post(
                    std::bind(realHandler, e.code(), std::make_pair(std::string(), std::string())));
            }
        });

    return init.result.get();
}

} // namespace baromesh

#endif