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

class DaemonServer {
    using Tcp = boost::asio::ip::tcp;

    using MethodIn = rpc::MethodIn<barobo::Daemon>;
    using MethodResult = rpc::MethodResult<barobo::Daemon>;

    using TcpPolyServer = rpc::asio::TcpPolyServer;

    using ProxyPair = std::pair<ZigbeeClient, TcpPolyServer>;

public:
    DaemonServer (TcpPolyServer& server, Dongle& dongle)
        : mServer(server)
        , mDongle(dongle)
        , mResolver(mServer.get_io_service())
        , mStrand(mServer.get_io_service())
    {}

    template <class In, class Result = typename rpc::ResultOf<In>::type>
    Result fire (In&& x) {
        return onFire(std::forward<In>(x));
    }

    MethodResult::getRobotTcpEndpoint onFire (MethodIn::getRobotTcpEndpoint args) {
        auto serialId = std::string(args.serialId.value);
        BOOST_LOG(mLog) << "firing barobo.Daemon.getRobotTcpEndpoint(" << serialId << ")";
        
        MethodResult::getRobotTcpEndpoint result = decltype(result)();
        try {
            auto iter = mRobotProxies.find(serialId);
            Tcp::endpoint endpoint;
            if (mRobotProxies.end() == iter) {
                // Bind to a random, free local port.
                Tcp::resolver::query query {
                    "127.0.0.1", "", boost::asio::ip::resolver_query_base::flags(0)
                };
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
                    std::bind(&rpc::asio::forwardBroadcastsCoroutine<ZigbeeClient, TcpPolyServer>,
                        std::ref(proxyClient), std::ref(proxyServer), _1));
                // FIXME remove this proxy after forwardRequestsCoroutine completes
                boost::asio::spawn(mStrand,
                    std::bind(&rpc::asio::forwardRequestsCoroutine<ZigbeeClient, TcpPolyServer>,
                        std::ref(proxyClient), std::ref(proxyServer), _1));
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
    mutable boost::log::sources::logger mLog;

    TcpPolyServer& mServer;
    Dongle& mDongle;

    Tcp::resolver mResolver;
    boost::asio::io_service::strand mStrand;

    std::map<std::string, ProxyPair> mRobotProxies;
};

} // namespace baromesh

#endif