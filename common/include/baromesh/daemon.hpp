#ifndef BAROMESH_DAEMON_HPP
#define BAROMESH_DAEMON_HPP

#include "rpc/asio/tcpclient.hpp"
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

#undef M_PI
#define M_PI 3.14159265358979323846

namespace baromesh {

namespace {

template <class T>
T degToRad (T x) { return T(double(x) * M_PI / 180.0); }

template <class T>
T radToDeg (T x) { return T(double(x) * 180.0 / M_PI); }

std::string daemonHostName () {
    return "127.0.0.1";
}

std::string daemonServiceName () {
    return "42000";
}

}

using StringPair = std::pair<std::string, std::string>;
typedef void ResolveSerialIdHandlerSignature(boost::system::error_code, StringPair);
using ResolveSerialIdHandler = std::function<ResolveSerialIdHandlerSignature>;

template <class Duration, class Handler>
BOOST_ASIO_INITFN_RESULT_TYPE(Handler, ResolveSerialIdHandlerSignature)
asyncResolveSerialId (rpc::asio::TcpClient& daemon, std::string serialId, Duration&& timeout, Handler&& handler) {
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
                    BOOST_LOG(log) << "Error firing resolveSerialId: " << ec.message();
                    throw boost::system::system_error(ec);
                }

                if (result.status) {
                    ec = Status(result.status);
                    BOOST_LOG(log) << "resolveSerialId resulted in error: " << ec.message();
                    throw boost::system::system_error(ec);
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