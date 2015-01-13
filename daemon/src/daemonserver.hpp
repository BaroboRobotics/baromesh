#ifndef BAROMESH_DAEMON_DAEMONSERVER_HPP
#define BAROMESH_DAEMON_DAEMONSERVER_HPP

#include "gen-daemon.pb.hpp"

#include "basicdongle.hpp"
#include "baromesh/dongledevicepath.hpp"

#include "rpc/asio/client.hpp"
#include "rpc/asio/forwardcoroutines.hpp"
#include "rpc/asio/server.hpp"
#include "rpc/asio/tcppolyserver.hpp"

#include "rpc/message.hpp"
#include "rpc/version.hpp"

#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/use_future.hpp>

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <cstring>

namespace baromesh {

using boost::asio::use_future;

using SerialMessageQueue = sfp::asio::MessageQueue<boost::asio::serial_port>;
using SerialClient = rpc::asio::Client<SerialMessageQueue>;
using Dongle = baromesh::BasicDongle<SerialClient>;
using ZigbeeClient = rpc::asio::Client<Dongle::MessageQueue>;

static const int kDongleBaudRate = 230400;

// One of the daemon's responsibilities is to acquire and communicate with the
// dongle. Dongle acquisition involves the following steps:
//
//   - Query the operating system for the dongle's serial device (e.g., COM3).
//   - Open the dongle's serial device.
//   - Sleep for kDongleSettleTimeAfterOpen duration.
//   - Set dongle serial device options (baud rate, parity, etc.).
//   - Conduct an SFP handshake.
//   - Conduct an RPC connection request, allowing the dongle
//     kDongleConnectTimeout duration to reply.
//
// If at any point during the acquisition process the daemon encounters an
// error, the process is restarted after kDongleDevicePathPollTimeout duration.
//
// If a read or write error is encountered after the dongle has been acquired,
// the daemon will attempt to reacquire the dongle after
// kDongleDowntimeAfterError duration.

// The amount of time to wait between unsuccessful attempts to acquire the
// dongle.
static const std::chrono::milliseconds kDongleDevicePathPollTimeout { 500 };

// The amount of time we give the dongle to respond to our RPC connect request.
static const std::chrono::milliseconds kDongleConnectTimeout { 1000 };

// How long we wait after an I/O error occurs on the dongle before trying to
// reacquire the dongle.
static const std::chrono::milliseconds kDongleDowntimeAfterError { 500 };

// How long we pause after opening the dongle's device path before setting the
// serial line options. Mac serial ports require some strategic timing
// ninjitsu in order to work, adjust this value as necessary.
static const std::chrono::milliseconds kDongleSettleTimeAfterOpen { 500 };

class DaemonServerImpl : public std::enable_shared_from_this<DaemonServerImpl> {
    using Tcp = boost::asio::ip::tcp;
    using TcpPolyServer = rpc::asio::TcpPolyServer;

    struct ProxyData {
        ProxyData (boost::asio::io_service& ios, Tcp::endpoint endpoint, boost::log::sources::logger log)
            : client(ios, log)
            , server(ios, endpoint, log)
        {}

        ZigbeeClient client;
        TcpPolyServer server;
    };

public:
    using MethodIn = rpc::MethodIn<barobo::Daemon>;
    using MethodResult = rpc::MethodResult<barobo::Daemon>;
    using Broadcast = rpc::Broadcast<barobo::Daemon>;

    DaemonServerImpl (boost::asio::io_service& ios, boost::log::sources::logger log)
        : mIos(ios)
        , mStrand(ios)
        , mResolver(ios)
        , mServer(mIos, *mResolver.resolve(decltype(mResolver)::query("127.0.0.1", "42000")), log)
        , mDongleTimer(ios)
        , mLog(log)
    {}

    void init () {
        auto self = this->shared_from_this();
        mStrand.post([self, this] {
            BOOST_LOG(mLog) << "Waiting for dongle";
            cycleDongleImpl(std::chrono::seconds(0));
        });
    }

    void destroy () {
        boost::system::error_code ec;
        close(ec);
    }

    void close (boost::system::error_code& ec) {
        auto lEc = ec;
        mResolver.cancel(); // resolver has no ec overload, strange

        if (mDongle) {
            mDongle->close(lEc);
            if (lEc) {
                ec = lEc;
            }
        }
        mServer.close(lEc);
        if (lEc) {
            ec = lEc;
        }

        mDongleTimer.cancel(lEc);
        if (lEc) {
            ec = lEc;
        }

        std::lock_guard<std::mutex> lock { mRobotProxiesMutex };
        for (auto& kv : mRobotProxies) {
            auto& proxy = *kv.second;
            proxy.client.close(lEc);
            if (lEc) {
                ec = lEc;
            }
            proxy.server.close(lEc);
            if (lEc) {
                ec = lEc;
            }
        }
    }

    void run () {
        try {
            while (true) {
                rpc::asio::asyncRunServer<barobo::Daemon>(mServer, *this, use_future).get();
            }
        }
        catch (std::exception& e) {
            BOOST_LOG(mLog) << "Exception in DaemonServer::run: " << e.what();
            return;
        }
    }

    template <class In, class Result = typename rpc::ResultOf<In>::type>
    Result fire (In&& x) {
        return onFire(std::forward<In>(x));
    }

    MethodResult::cycleDongle onFire (MethodIn::cycleDongle args) {
        BOOST_LOG(mLog) << "firing barobo.Daemon.cycleDongle("
                        << args.seconds << " seconds)";
        cycleDongleImpl(std::chrono::seconds(args.seconds));
        return {};
    }

    MethodResult::resolveSerialId onFire (MethodIn::resolveSerialId args) {
        auto serialId = std::string(args.serialId.value);
        BOOST_LOG(mLog) << "firing barobo.Daemon.resolveSerialId(" << serialId << ")";

        MethodResult::resolveSerialId result = decltype(result)();
        try {
            if (!mDongle) {
                throw std::runtime_error("no dongle present");
            }

            BOOST_LOG(mLog) << "searching for proxy for " << serialId;
            std::lock_guard<std::mutex> lock { mRobotProxiesMutex };
            auto iter = mRobotProxies.find(serialId);
            Tcp::endpoint endpoint;
            if (mRobotProxies.end() == iter) {
                BOOST_LOG(mLog) << "No proxy exists, buiding resolver query";
                // Bind to a random, free local port.
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
                auto proxy = std::make_shared<ProxyData>(mIos, endpoint, pxLog);
                bool success;
                std::tie(iter, success) = mRobotProxies.insert(std::make_pair(serialId, proxy));
                assert(success);

                auto& proxyClient = iter->second->client;
                auto& proxyServer = iter->second->server;

                proxyClient.messageQueue().setRoute(*mDongle, serialId);

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
        std::lock_guard<std::mutex> lock { mRobotProxiesMutex };
        auto nErased = mRobotProxies.erase(serialId);
        BOOST_LOG(mLog) << "Proxy for " << serialId
                        << (nErased ? " erased; " : " does not exist! ")
                        << mRobotProxies.size() << " proxies remaining";
    }

    template <class Duration>
    void cycleDongleImpl (Duration&& timeout) {
        if (mDongle) {
            mDongle->close();
            mDongle.reset();
        }
        mDongleTimer.cancel();
        mDongleTimer.expires_from_now(std::forward<Duration>(timeout));
        mDongleTimer.async_wait(mStrand.wrap(
            std::bind(&DaemonServerImpl::handleCycleDongleStepOne, this->shared_from_this(), _1)));
    }

    void handleCycleDongleStepOne (boost::system::error_code ec) {
        if (!ec) {
            try {
                // Find the dongle's device path, open it, and start the SFP
                // handshake.
                auto devicePath = dongleDevicePath();
                BOOST_LOG(mLog) << "Dongle detected at " << devicePath;

                boost::log::sources::logger dongleClLog;
                dongleClLog.add_attribute("Title", boost::log::attributes::constant<std::string>("DONGLE-CL"));

                auto dongle = std::make_shared<Dongle>(mIos, dongleClLog);
                dongle->client().messageQueue().stream().open(devicePath);

                mDongleTimer.expires_from_now(kDongleSettleTimeAfterOpen);
                mDongleTimer.async_wait(mStrand.wrap(
                    std::bind(&DaemonServerImpl::handleCycleDongleStepTwo,
                        this->shared_from_this(), dongle, _1)));
            }
            catch (std::exception& e) {
                cycleDongleImpl(kDongleDevicePathPollTimeout);
            }
        }
    }

    void handleCycleDongleStepTwo (std::shared_ptr<Dongle> dongle, boost::system::error_code ec) {
        if (!ec) {
            try {
                using Option = boost::asio::serial_port_base;
                auto& stream = dongle->client().messageQueue().stream();

                stream.set_option(Option::baud_rate(kDongleBaudRate));
                stream.set_option(Option::character_size(8));
                stream.set_option(Option::parity(Option::parity::none));
                stream.set_option(Option::stop_bits(Option::stop_bits::one));
                stream.set_option(Option::flow_control(Option::flow_control::none));

#ifdef __MACH__
                auto handle = stream.native_handle();
                ::write(handle, nullptr, 0);
#endif

                dongle->client().messageQueue().asyncHandshake(mStrand.wrap(
                    std::bind(&DaemonServerImpl::handleCycleDongleStepThree,
                        this->shared_from_this(), dongle, _1)));
            }
            catch (std::exception& e) {
                cycleDongleImpl(kDongleDevicePathPollTimeout);
            }
        }
    }

    void handleCycleDongleStepThree (std::shared_ptr<Dongle> dongle, boost::system::error_code ec) {
        if (!ec) {
            rpc::asio::asyncConnect<barobo::Dongle>(dongle->client(), kDongleConnectTimeout, mStrand.wrap(
                std::bind(&DaemonServerImpl::handleCycleDongleStepFour,
                    this->shared_from_this(), dongle, _1)));
        }
        else if (boost::asio::error::operation_aborted != ec) {
            cycleDongleImpl(kDongleDevicePathPollTimeout);
        }
    }

    void handleCycleDongleStepFour (std::shared_ptr<Dongle> dongle,
                                     boost::system::error_code ec) {
        if (!ec) {
            mDongle.reset(new Dongle{std::move(*dongle)});
            setDongleIoTraps();
            asyncBroadcast(mServer, Broadcast::dongleDetected{},
                [] (boost::system::error_code ec) {
                    if (ec && boost::asio::error::operation_aborted != ec) {
                        boost::log::sources::logger log;
                        BOOST_LOG(log) << "dongleDetected broadcast completed"
                                       << " with " << ec.message();
                    }
                });
        }
        else if (boost::asio::error::operation_aborted != ec) {
            // TODO propagate VERSION_MISMATCH via dongleDetected broadcast?
            cycleDongleImpl(kDongleDevicePathPollTimeout);
        }
    }

    void setDongleIoTraps () {
        assert(mDongle);

        // Now that we're all connected up, queue up an asynchronous operation
        // that should in theory never complete. The only time this code should
        // execute, then, is when there's a dongle read error. In that case, we
        // want to wait for a dongle to appear in the OS environment again.
        auto mqLog = mLog;
        mqLog.add_attribute("SerialId", boost::log::attributes::constant<std::string>("...."));

        auto buf = std::make_shared<uint8_t>();
        auto nullMq = std::make_shared<Dongle::MessageQueue>(mIos, mqLog);

        auto self = this->shared_from_this();
        auto resetDongle = [self, this] (boost::system::error_code ec) {
            BOOST_LOG(mLog) << "Resetting dongle because: " << ec.message();
            if (boost::asio::error::operation_aborted != ec) {
                assert(mDongle);
                cycleDongleImpl(kDongleDowntimeAfterError);
            }
        };

        nullMq->setRoute(*mDongle, "....");
        nullMq->asyncReceive(boost::asio::buffer(buf.get(), 1), mStrand.wrap(
            [resetDongle, nullMq] (boost::system::error_code ec, size_t) {
                resetDongle(ec);
            }));

        // So that takes care of linux. Windows XP's implementation of IOCP is
        // too stupid to notify us when a serial read fails because of a device
        // unplugged, so ping the dongle periodically.
        mDongle->client().messageQueue().asyncKeepalive(mStrand.wrap(
            [resetDongle] (boost::system::error_code ec) {
                resetDongle(ec);
            }));
    }

    boost::asio::io_service& mIos;
    boost::asio::io_service::strand mStrand;

    Tcp::resolver mResolver;

    TcpPolyServer mServer;
    std::unique_ptr<Dongle> mDongle;
    boost::asio::steady_timer mDongleTimer;

    std::map<std::string, std::shared_ptr<ProxyData>> mRobotProxies;
    std::mutex mRobotProxiesMutex;

    mutable boost::log::sources::logger mLog;
};

class DaemonServer {
public:
    template <class... Args>
    DaemonServer (Args&&... args)
        : mImpl(std::make_shared<DaemonServerImpl>(std::forward<Args>(args)...))
    {
        mImpl->init();
    }

    // noncopyable
    DaemonServer (const DaemonServer&) = delete;
    DaemonServer& operator= (const DaemonServer&) = delete;

    ~DaemonServer () {
        mImpl->destroy();
    }

    void close (boost::system::error_code& ec) {
        mImpl->close(ec);
    }

    void run () {
        mImpl->run();
    }

private:
    std::shared_ptr<DaemonServerImpl> mImpl;
};

} // namespace baromesh

#endif
