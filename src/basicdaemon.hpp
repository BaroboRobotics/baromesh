#ifndef BAROMESH_BASICDAEMON_HPP
#define BAROMESH_BASICDAEMON_HPP

#include "gen-daemon.pb.hpp"

#include "baromesh/system_error.hpp"

#include <boost/asio.hpp>

#include <boost/log/common.hpp>
#include <boost/log/sources/logger.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <utility>

namespace baromesh {

using namespace std::placeholders;

using GetRobotTcpEndpointHandlerSignature = void(boost::system::error_code, boost::asio::ip::tcp::resolver::iterator);
using GetRobotTcpEndpointHandler = std::function<GetRobotTcpEndpointHandlerSignature>;

template <class C>
class DaemonImpl : public std::enable_shared_from_this<DaemonImpl<C>> {
    using MethodIn = rpc::MethodIn<barobo::Daemon>;
    using MethodResult = rpc::MethodResult<barobo::Daemon>;

public:
    template <class... Args>
    explicit DaemonImpl (Args&&... args)
        : mClient(std::forward<Args>(args)...)
        , mResolver(mClient.get_io_service())
    {}

    C& client () { return mClient; }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler, GetRobotTcpEndpointHandlerSignature)
    asyncGetRobotTcpEndpoint (std::string serialId, Handler&& handler) {
        boost::asio::detail::async_result_init<
            Handler, GetRobotTcpEndpointHandlerSignature
        > init { std::forward<Handler>(handler) };

        assert(4 == serialId.size());
        MethodIn::getRobotTcpEndpoint args = decltype(args)();

        strncpy(args.serialId.value, serialId.data(), 4);
        args.serialId.value[4] = 0;

        asyncFire(mClient, args, std::chrono::seconds(2),
            std::bind(&DaemonImpl::handleGetRobotTcpEndpoint,
                this->shared_from_this(), init.handler, _1, _2));

        return init.result.get();
    }

private:
    void handleGetRobotTcpEndpoint (GetRobotTcpEndpointHandler handler,
        boost::system::error_code ec,
        MethodResult::getRobotTcpEndpoint result) {
        try {
            if (ec) {
                BOOST_LOG(mLog) << "getRobotTcpEndpoint reported error: " << ec.message();
                throw boost::system::system_error(ec);
            }

            if (!result.has_endpoint) {
                BOOST_LOG(mLog) << "getRobotTcpEndpoint result has no endpoint";
                throw boost::system::system_error(Status::NO_ROBOT_ENDPOINT);
            }

            auto port = uint16_t(result.endpoint.port);
            if (port != result.endpoint.port) {
                throw boost::system::system_error(Status::PORT_OUT_OF_RANGE);
            }

            BOOST_LOG(mLog) << "resolving endpoint from (\""
                            << std::string(result.endpoint.address)
                            << "\", " << port << ")";

            using std::to_string;
            typename decltype(mResolver)::query query {
                std::string(result.endpoint.address),
                to_string(port)
            };

            auto self = this->shared_from_this();
            mResolver.async_resolve(query,
                [self, this, handler]
                (boost::system::error_code ec, boost::asio::ip::tcp::resolver::iterator iter) {
                    if (!ec) {
                        BOOST_LOG(mLog) << "Endpoint is " << iter->endpoint();
                        mClient.get_io_service().post(
                            std::bind(handler, boost::system::error_code(), iter));
                    }
                    else {
                        mClient.get_io_service().post(
                            std::bind(handler, boost::system::error_code(), iter));
                    }
                });
        }
        catch (boost::system::system_error& e) {
            BOOST_LOG(mLog) << "Error in handleGetRobotTcpEndpoint: " << e.what();
            mClient.get_io_service().post(
                std::bind(handler, e.code(), boost::asio::ip::tcp::resolver::iterator()));
        }
    }

    mutable boost::log::sources::logger mLog;

    C mClient;
    boost::asio::ip::tcp::resolver mResolver;
};

template <class C>
class BasicDaemon {
public:
    using Client = C;
    using Impl = DaemonImpl<Client>;

    template <class... Args>
    explicit BasicDaemon (Args&&... args)
        : mImpl(std::make_shared<Impl>(std::forward<Args>(args)...))
    {}

    // noncopyable
    BasicDaemon (const BasicDaemon&) = delete;
    BasicDaemon& operator= (const BasicDaemon&) = delete;

    Client& client () { return mImpl->client(); }
    boost::asio::io_service& get_io_service () { return client().get_io_service(); }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler, GetRobotTcpEndpointHandlerSignature)
    asyncGetRobotTcpEndpoint (std::string serialId, Handler&& handler) {
        return mImpl->asyncGetRobotTcpEndpoint(serialId, std::forward<Handler>(handler));
    }

private:
    std::shared_ptr<Impl> mImpl;
};

}

#endif