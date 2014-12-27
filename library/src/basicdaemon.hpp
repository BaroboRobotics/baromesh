#ifndef BAROMESH_BASICDAEMON_HPP
#define BAROMESH_BASICDAEMON_HPP

#include "gen-daemon.pb.hpp"

#include "baromesh/system_error.hpp"

#include <boost/asio/async_result.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <utility>

namespace baromesh {

using namespace std::placeholders;

using ResolveSerialIdHandlerSignature = void(boost::system::error_code, boost::asio::ip::tcp::resolver::iterator);
using ResolveSerialIdHandler = std::function<ResolveSerialIdHandlerSignature>;

template <class C>
class DaemonImpl : public std::enable_shared_from_this<DaemonImpl<C>> {
    using MethodIn = rpc::MethodIn<barobo::Daemon>;
    using MethodResult = rpc::MethodResult<barobo::Daemon>;

public:
    explicit DaemonImpl (boost::asio::io_service& ios, boost::log::sources::logger log)
        : mClient(ios, log)
        , mResolver(mClient.get_io_service())
        , mLog(log)
    {}

    C& client () { return mClient; }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler, ResolveSerialIdHandlerSignature)
    asyncResolveSerialId (std::string serialId, Handler&& handler) {
        boost::asio::detail::async_result_init<
            Handler, ResolveSerialIdHandlerSignature
        > init { std::forward<Handler>(handler) };

        assert(4 == serialId.size());
        MethodIn::resolveSerialId args = decltype(args)();

        strncpy(args.serialId.value, serialId.data(), 4);
        args.serialId.value[4] = 0;

        asyncFire(mClient, args, std::chrono::seconds(2),
            std::bind(&DaemonImpl::handleResolveSerialId,
                this->shared_from_this(), init.handler, _1, _2));

        return init.result.get();
    }

private:
    void handleResolveSerialId (ResolveSerialIdHandler handler,
        boost::system::error_code ec,
        MethodResult::resolveSerialId result) {
        try {
            if (ec) {
                BOOST_LOG(mLog) << "resolveSerialId reported error: " << ec.message();
                throw boost::system::system_error(ec);
            }

            if (!result.has_endpoint) {
                BOOST_LOG(mLog) << "resolveSerialId result has no endpoint";
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

    C mClient;
    boost::asio::ip::tcp::resolver mResolver;

    mutable boost::log::sources::logger mLog;
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
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler, ResolveSerialIdHandlerSignature)
    asyncResolveSerialId (std::string serialId, Handler&& handler) {
        return mImpl->asyncResolveSerialId(serialId, std::forward<Handler>(handler));
    }

private:
    std::shared_ptr<Impl> mImpl;
};

}

#endif