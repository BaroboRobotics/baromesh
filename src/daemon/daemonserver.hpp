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

    using MethodIn = rpc::MethodIn<barobo::Daemon>;
    using MethodResult = rpc::MethodResult<barobo::Daemon>;

    using TcpPolyServer = rpc::asio::TcpPolyServer;

    using ProxyPair = std::pair<ZigbeeClient, TcpPolyServer>;

public:
    DaemonServerImpl (TcpPolyServer& server, Dongle& dongle)
        : mServer(server)
        , mDongle(dongle)
        , mResolver(mServer.get_io_service())
        , mStrand(mServer.get_io_service())
    {}

    void destroy () {
        mResolver.cancel(); // resolver has no ec overload, strange
        boost::system::error_code ec;
        for (auto& kv : mRobotProxies) {
            auto& proxyClient = kv.second.first;
            auto& proxyServer = kv.second.second;
            proxyClient.cancel(ec);
            proxyServer.cancel(ec);
        }
    }

    MethodResult::resolveSerialId onFire (MethodIn::resolveSerialId args) {
        auto serialId = std::string(args.serialId.value);
        BOOST_LOG(mLog) << "firing barobo.Daemon.resolveSerialId(" << serialId << ")";
        
        MethodResult::resolveSerialId result = decltype(result)();
        try {
            auto iter = mRobotProxies.find(serialId);
            Tcp::endpoint endpoint;
            if (mRobotProxies.end() == iter) {
                // Bind to a random, free local port.
                Tcp::resolver::query query {
                    "127.0.0.1", "", boost::asio::ip::resolver_query_base::flags(0)
                };
                // FIXME this resolve is synchronous. Make it asynchronous.
                // This will require us to enable asynchronous method invocation in
                // ribbon-bridge.
                endpoint = mResolver.resolve(query)->endpoint();

                BOOST_LOG(mLog) << "Starting new robot proxy on " << endpoint;
                bool success;
                std::tie(iter, success) = mRobotProxies.emplace(std::piecewise_construct,
                    std::forward_as_tuple(serialId),
                    std::forward_as_tuple(std::piecewise_construct,
                        std::forward_as_tuple(std::ref(mDongle), serialId),
                        std::forward_as_tuple(mServer.get_io_service(), endpoint)));
                assert(success);

                auto& proxyClient = iter->second.first;
                auto& proxyServer = iter->second.second;

                boost::asio::spawn(mStrand,
                    std::bind(&DaemonServerImpl::proxyCoroutine,
                        this->shared_from_this(), serialId, _1));
            }

            endpoint = iter->second.second.endpoint();
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
    void proxyCoroutine (std::string serialId, boost::asio::yield_context yield) {
        try {
            auto iter = mRobotProxies.find(serialId);
            if (iter != mRobotProxies.end()) {
                auto& proxyClient = iter->second.first;
                auto& proxyServer = iter->second.second;

                boost::asio::spawn(mStrand,
                    std::bind(&rpc::asio::forwardBroadcastsCoroutine<ZigbeeClient, TcpPolyServer>,
                        std::ref(proxyClient), std::ref(proxyServer), _1));

                rpc::asio::forwardRequestsCoroutine(proxyClient, proxyServer, yield);
                BOOST_LOG(mLog) << "DaemonServerImpl::proxyCoroutine completed normally";
            }
            else {
                BOOST_LOG(mLog) << "DaemonServerImpl::proxyCoroutine started for nonexistent connection "
                                << serialId << ", ignoring";
            }
        }
        catch (boost::system::system_error& e) {
            BOOST_LOG(mLog) << "Error in DaemonServerImpl::proxyCoroutine for " << serialId
                            << ": " << e.what();
        }
        BOOST_LOG(mLog) << "Destroying proxy for " << serialId;
        mRobotProxies.erase(serialId);
    }

    mutable boost::log::sources::logger mLog;

    TcpPolyServer& mServer;
    Dongle& mDongle;

    Tcp::resolver mResolver;
    boost::asio::io_service::strand mStrand;

    std::map<std::string, ProxyPair> mRobotProxies;
};

class DaemonServer {
public:
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