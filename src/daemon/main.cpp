#include "daemonserver.hpp"
#include "dongledevicepath.hpp"

#include "sfp/asio/messagequeue.hpp"

#include "rpc/message.hpp"
#include "rpc/version.hpp"
#include "rpc/asio/client.hpp"
#include "rpc/asio/server.hpp"
#include "rpc/asio/tcppolyserver.hpp"

#include "util/hexdump.hpp"
#include "util/monospawn.hpp"

#include <boost/asio.hpp>
#include <boost/asio/spawn.hpp>

#include <functional>
#include <iostream>
#include <string>

using namespace std::placeholders;

using Tcp = boost::asio::ip::tcp;
using TcpMessageQueue = sfp::asio::MessageQueue<Tcp::socket>;

static const int kBaudRate = 230400;

void dongleCoroutine (boost::asio::io_service& ios, boost::asio::yield_context yield) {
    boost::log::sources::logger log;
    try {
        boost::asio::steady_timer donglePollTimer { ios };
        std::string devicePath;
        boost::system::error_code ec;
        devicePath = baromesh::dongleDevicePath(ec);
        while (ec) {
            donglePollTimer.expires_from_now(std::chrono::milliseconds(200));
            donglePollTimer.async_wait(yield);
            devicePath = baromesh::dongleDevicePath(ec);
        }

        BOOST_LOG(log) << "Dongle detected at " << devicePath;

        baromesh::Dongle dongle { ios };

        auto& stream = dongle.client().messageQueue().stream();
        stream.open(devicePath);
        stream.set_option(boost::asio::serial_port_base::baud_rate(kBaudRate));

        dongle.client().messageQueue().asyncHandshake(yield);
        asyncConnect(dongle.client(), std::chrono::milliseconds(100), yield);

        Tcp::resolver resolver { ios };
        auto iter = resolver.async_resolve(std::string("42000"), yield);
        
        rpc::asio::TcpPolyServer server  { ios, *iter };

        rpc::asio::TcpPolyServer::RequestId requestId;
        barobo_rpc_Request request;

        while (true) {
            BOOST_LOG(log) << "DaemonServer awaiting connection";
            // Refuse requests with Status::NOT_CONNECTED until we get a CONNECT
            // request.
            std::tie(requestId, request) = processRequestsCoro(server,
                std::bind(&rpc::asio::rejectIfNotConnectCoro<rpc::asio::TcpPolyServer>,
                    std::ref(server), _1, _2, _3), yield);

            // Reply with barobo::Daemon's version information.
            asyncReply(server, requestId, rpc::ServiceInfo::create<barobo::Daemon>(), yield);
            BOOST_LOG(log) << "DaemonServer connection received";

            baromesh::DaemonServer daemon { server, dongle };

            std::tie(requestId, request) = processRequestsCoro(server,
                std::bind(&rpc::asio::serveIfNotDisconnectCoro<rpc::asio::TcpPolyServer, barobo::Daemon, baromesh::DaemonServer>,
                    std::ref(server), std::ref(daemon), _1, _2, _3), yield);

            BOOST_LOG(log) << "DaemonServer received disconnection request";
            asyncReply(server, requestId, rpc::Status::OK, yield);
        }
    }
    catch (boost::system::system_error& e) {
        BOOST_LOG(log) << "Error in dongle coroutine: " << e.what();
    }
    BOOST_LOG(log) << "Restarting dongle coroutine";
    boost::asio::spawn(ios, std::bind(&dongleCoroutine, std::ref(ios), _1));
}

int main (int argc, char** argv) try {
    util::Monospawn sentinel { "baromeshd", std::chrono::seconds(1) };
    // TODO initialize logging core
    boost::log::sources::logger log;
    boost::asio::io_service ios;

#if 0 // TODO
    // Make a deadman switch to stop the daemon if we get an exclusive
    // producer lock.
    util::asio::TmpFileLock producerLock { ios };
    producerLock.asyncLock([&] (boost::system::error_code ec) {
        if (!ec) {
            BOOST_LOG(log) << "No more baromesh producers, exiting";
            ios.stop();
        }
    });
#endif

    boost::asio::spawn(ios, std::bind(&dongleCoroutine, std::ref(ios), _1));

    boost::system::error_code ec;
    auto nHandlers = ios.run(ec);
    BOOST_LOG(log) << "Event loop executed " << nHandlers << " handlers -- " << ec.message();
}
catch (util::Monospawn::DuplicateProcess& e) {
    boost::log::sources::logger log;
    BOOST_LOG(log) << "baromeshd already running, exiting";
}
catch (std::exception& e) {
    boost::log::sources::logger log;
    BOOST_LOG(log) << "baromeshd caught some other exception: " << e.what();
}