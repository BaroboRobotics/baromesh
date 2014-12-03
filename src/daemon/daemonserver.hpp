#ifndef BAROMESH_DAEMON_DAEMONSERVER_HPP
#define BAROMESH_DAEMON_DAEMONSERVER_HPP

#include "gen-daemon.pb.hpp"
#include "dongle.hpp"

#include "rpc/asio/forwardcoroutines.hpp"
#include "rpc/asio/tcppolyserver.hpp"

#include <boost/log/common.hpp>
#include <boost/log/sources/logger.hpp>

#include <memory>
#include <string>
#include <utility>

#include <cstring>

namespace baromesh {

class DaemonServerImpl : public std::enable_shared_from_this<DaemonServerImpl> {
    using Tcp = boost::asio::ip::tcp;
    using TcpPolyServer = rpc::asio::TcpPolyServer;

    struct ProxyData {
        ProxyData (boost::asio::io_service& ios, boost::log::sources::logger log)
            : client(ios, log)
            , server(ios, log)
        {}

        ZigbeeClient client;
        TcpPolyServer server;
    };

public:
    using Interface = barobo::Daemon;
    using MethodIn = rpc::MethodIn<Interface>;
    using MethodResult = rpc::MethodResult<Interface>;

    DaemonServerImpl (TcpPolyServer& server, Dongle& dongle, boost::log::sources::logger log)
        : mServer(server)
        , mDongle(dongle)
        , mResolver(mServer.get_io_service())
        , mStrand(mServer.get_io_service())
        , mLog(log)
    {}

    void destroy () {
        boost::system::error_code ec;
        close(ec);
    }

    void close (boost::system::error_code& ec) {
        mResolver.cancel(); // resolver has no ec overload, strange
        for (auto& kv : mRobotProxies) {
            auto& proxyClient = kv.second->client;
            auto& proxyServer = kv.second->server;
            proxyClient.close(ec);
            proxyServer.close(ec);
        }
    }

    MethodResult::resolveSerialId onFire (MethodIn::resolveSerialId args) {
        auto serialId = std::string(args.serialId.value);
        BOOST_LOG(mLog) << "firing barobo.Daemon.resolveSerialId(" << serialId << ")";
        
        MethodResult::resolveSerialId result = decltype(result)();
        try {
            BOOST_LOG(mLog) << "searching for proxy for " << serialId;
            auto iter = mRobotProxies.find(serialId);
            Tcp::endpoint endpoint;
            if (mRobotProxies.end() == iter) {
                BOOST_LOG(mLog) << "No proxy exists, buiding resolver query";
                // Bind to a random, free local port.
                //Tcp::resolver::query query {
                //    "127.0.0.1", "", boost::asio::ip::resolver_query_base::flags(0)
                //};
                // FIXME this resolve is synchronous. Make it asynchronous.
                // This will require us to enable asynchronous method invocation in
                // ribbon-bridge.
                BOOST_LOG(mLog) << "Resolving 127.0.0.1:any port";
                auto epIter = mResolver.resolve(decltype(mResolver)::query("127.0.0.1", "",
                    boost::asio::ip::resolver_query_base::flags(0)));
                BOOST_LOG(mLog) << "Resolved to iterator";
                endpoint = epIter->endpoint();

                BOOST_LOG(mLog) << "Starting new robot proxy on " << endpoint;
                boost::log::sources::logger pxLog;
                pxLog.add_attribute("Title", boost::log::attributes::constant<std::string>("PROXY"));
                pxLog.add_attribute("SerialId", boost::log::attributes::constant<std::string>(serialId));
                auto proxy = std::make_shared<ProxyData>(mServer.get_io_service(), pxLog);
                bool success;
                std::tie(iter, success) = mRobotProxies.insert(std::make_pair(serialId, proxy));
                assert(success);

                auto& proxyClient = iter->second->client;
                auto& proxyServer = iter->second->server;

                proxyClient.messageQueue().setRoute(mDongle, serialId);
                proxyServer.listen(endpoint);

                asyncRunProxy(proxyClient, proxyServer, mStrand.wrap(
                    std::bind(&DaemonServerImpl::handleProxyFinished,
                        this->shared_from_this(), serialId, proxy, _1)));
            }

            endpoint = iter->second->server.endpoint();
            BOOST_LOG(mLog) << "Using proxy server for " << serialId
                            << " at " << endpoint;

            if (endpoint.address().to_string().size() < sizeof(result.endpoint.address)) {
                strncpy(result.endpoint.address,
                    endpoint.address().to_string().data(),
                    sizeof(result.endpoint.address) - 1);
                result.endpoint.address[sizeof(result.endpoint.address) - 1] = 0;
                result.endpoint.port = endpoint.port();
                result.has_endpoint = true;
            }
            else {
                throw std::runtime_error("address buffer overflow");
            }
        }
        catch (std::exception& e) {
            BOOST_LOG(mLog) << "Error (re)starting proxy server for "
                            << std::string(serialId) << ": " << e.what();
            result.has_endpoint = false;
        }
        return result;
    }

private:
    void handleProxyFinished (std::string serialId,
                              std::shared_ptr<ProxyData>,
                              boost::system::error_code ec) {
        BOOST_LOG(mLog) << "Proxy for " << serialId << " finished with " << ec.message();
        auto nErased = mRobotProxies.erase(serialId);
        BOOST_LOG(mLog) << "Proxy for " << serialId
                        << (nErased ? " erased; " : " does not exist! ")
                        << mRobotProxies.size() << " proxies remaining";
    }

    TcpPolyServer& mServer;
    Dongle& mDongle;

    Tcp::resolver mResolver;
    boost::asio::io_service::strand mStrand;

    // FIXME mRobotProxies needs to be protected by a mutex, unless we can
    // guarantee that all accesses take place on the same strand. That is
    // currently not the case, but since our io_services are all single-
    // threaded at the moment, there is an implicit strand which makes this
    // safe, for now. If we want to be able to support multiple threads
    // calling io_service::run simultaneously, this needs to be protected
    // by a mutex, or we need to implement asynchronous fire implementations,
    // which will permit us to use a strand to synchronize access to mRobotProxies.
    std::map<std::string, std::shared_ptr<ProxyData>> mRobotProxies;

    mutable boost::log::sources::logger mLog;
};

class DaemonServer {
public:
    using Interface = DaemonServerImpl::Interface;
    
    template <class... Args>
    DaemonServer (Args&&... args)
        : mImpl(std::make_shared<DaemonServerImpl>(std::forward<Args>(args)...))
    {}

    // noncopyable
    DaemonServer (const DaemonServer&) = delete;
    DaemonServer& operator= (const DaemonServer&) = delete;

    ~DaemonServer () {
        mImpl->destroy();
    }

    template <class In, class Result = typename rpc::ResultOf<In>::type>
    Result fire (In&& x) {
        return mImpl->onFire(std::forward<In>(x));
    }

private:
    std::shared_ptr<DaemonServerImpl> mImpl;
};

} // namespace baromesh

#endif