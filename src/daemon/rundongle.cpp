#include "breaker.hpp"
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
#include <boost/asio/use_future.hpp>

#include <boost/log/common.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/attributes/timer.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/console.hpp>

#include <boost/scope_exit.hpp>

#include <exception>
#include <functional>
#include <future>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

using namespace std::placeholders;

using Tcp = boost::asio::ip::tcp;
using TcpMessageQueue = sfp::asio::MessageQueue<Tcp::socket>;

using boost::asio::use_future;

static const int kBaudRate = 230400;

// return true for "run me again", false for stop
static bool runDongle (boost::asio::io_service& ios, Breaker& breaker) {
    std::chrono::milliseconds kDongleDevicePathPollTimeout { 500 };
    std::chrono::milliseconds kKeepaliveTimeout { 500 };
    boost::log::sources::logger log;
    log.add_attribute("Title", boost::log::attributes::constant<std::string>("DONGLECORO"));

    std::atomic<bool> continueRunning = { true };

    try {
        BOOST_LOG(log) << "Waiting for dongle";
        std::string devicePath;
        boost::asio::steady_timer timer { ios };
        {
            auto killswitch = Breaker::Killswitch(breaker, [&timer, &continueRunning] () {
                continueRunning = false;
                timer.cancel();
            });

            boost::system::error_code ec;
            devicePath = baromesh::dongleDevicePath(ec);
            while (ec) {
                timer.expires_from_now(kDongleDevicePathPollTimeout);
                timer.wait();
                devicePath = baromesh::dongleDevicePath(ec);
            }
        }

        BOOST_LOG(log) << "Dongle detected at " << devicePath;

        boost::log::sources::logger dongleClLog;
        dongleClLog.add_attribute("Title", boost::log::attributes::constant<std::string>("DONGLE-CL"));
        auto dongle = std::make_shared<baromesh::Dongle>(ios, dongleClLog);

        auto& stream = dongle->client().messageQueue().stream();
        using Option = boost::asio::serial_port_base;
        stream.open(devicePath);
        stream.set_option(Option::baud_rate(kBaudRate));
        stream.set_option(Option::character_size(8));
        stream.set_option(Option::parity(Option::parity::none));
        stream.set_option(Option::stop_bits(Option::stop_bits::one));
        stream.set_option(Option::flow_control(Option::flow_control::none));

        {
            auto killswitch = Breaker::Killswitch(breaker, [dongle, &continueRunning] () {
                continueRunning = false;
                dongle->close();
            });

            dongle->client().messageQueue().asyncHandshake(use_future).get();
            auto info = asyncConnect(dongle->client(), std::chrono::milliseconds(100), use_future).get();
            BOOST_LOG(log) << "Dongle has RPC version " << info.rpcVersion()
                           << ", interface version " << info.interfaceVersion();
#warning check for version mismatch
        }

        Tcp::resolver resolver { ios };
        auto iter = resolver.resolve(decltype(resolver)::query("127.0.0.1", "42000"));
        boost::log::sources::logger daemonSvLog;
        daemonSvLog.add_attribute("Title", boost::log::attributes::constant<std::string>("DAEMON-SV"));
        auto server = std::make_shared<rpc::asio::TcpPolyServer>(ios, daemonSvLog);
        server->listen(*iter);

        // Now that we're all connected up, queue up an asynchronous operation
        // that should in theory never complete. The only time this code should
        // execute, then, is when there's a dongle read error. In that case, we
        // want to shut the server operations down as well, and wait for a
        // dongle to appear in the OS environment again.
        auto mqLog = log;
        mqLog.add_attribute("SerialId", boost::log::attributes::constant<std::string>("...."));
        baromesh::Dongle::MessageQueue nullMq { ios, mqLog };
        nullMq.setRoute(*dongle, "....");

        auto stopTheWorld = [dongle, server] (boost::system::error_code ec) {
            boost::log::sources::logger log;
            BOOST_LOG(log) << "Stopping the world with: " << ec.message();
            server->close(ec);
            dongle->close(ec);
        };

        uint8_t buf[1];
        nullMq.asyncReceive(boost::asio::buffer(buf),
            [stopTheWorld] (boost::system::error_code ec, size_t) {
                boost::log::sources::logger log;
                if (!ec) {
                    BOOST_LOG(log) << "Actually read something from serial ID \"....\" . "
                                   << "That should be impossible.";
                }
                else {
                    BOOST_LOG(log) << "Dongle error: " << ec.message();
                }
                stopTheWorld(ec);
            });

        // So that takes care of linux. Windows XP's implementation of IOCP is
        // too stupid to notify us when a serial read fails because of a device
        // unplugged, so ping the dongle periodically.
        asyncKeepalive(dongle->client().messageQueue(), kKeepaliveTimeout, stopTheWorld);

        {
            auto killswitch = Breaker::Killswitch(breaker, [&continueRunning, stopTheWorld] () {
                continueRunning = false;
                boost::system::error_code ec;
                stopTheWorld(ec);
            });

            /* dko: This need not be a loop. If we run baromeshd as a service, the
             * service can terminate as soon as all connected entities disconnect.
             * */
            while (continueRunning) {
                BOOST_LOG(daemonSvLog) << "awaiting connection";
                baromesh::DaemonServer daemon { *server, *dongle, daemonSvLog };
                asyncRunServer(*server, daemon, use_future).get();
                BOOST_LOG(daemonSvLog) << "disconnected";
            }
        }
    }
    catch (boost::system::system_error& e) {
        BOOST_LOG(log) << "runDongle: " << e.what();
    }

    return continueRunning;
}



static void initializeLoggingCore () {
    namespace expr = boost::log::expressions;
    namespace keywords = boost::log::keywords;
    namespace attrs = boost::log::attributes;

    boost::log::add_common_attributes();

    auto core = boost::log::core::get();
    //core->add_global_attribute("Timeline", attrs::timer());
    core->add_global_attribute("Scope", attrs::named_scope());

    boost::log::formatter formatter =
        expr::stream
            //<< "[T+" << expr::attr<attrs::timer::value_type>("Timeline") << "]"
            << expr::attr<unsigned int>("LineID") << ": "
            << "[thread=" << expr::attr<attrs::current_thread_id::value_type>("ThreadID") << "]"
            << expr::if_ (expr::has_attr<std::string>("SerialId")) [
                expr::stream << "[robot=" << expr::attr<std::string>("SerialId") << "]"
            ] // .else_ []
            //<< "[" << expr::attr<attrs::named_scope::value_type>("Scope") << "]"
            << " " << expr::attr<std::string>("Title")
            << " " << expr::attr<std::string>("Protocol")
            << expr::if_ (expr::has_attr<std::string>("RequestId")) [
                expr::stream << "[RequestId=" << expr::attr<std::string>("RequestId") << "]"
            ]
            << " " << expr::smessage;

    boost::log::add_file_log(
        keywords::file_name = "baromeshd.log",
        keywords::auto_flush = true
    )->set_formatter(formatter);

    boost::log::add_console_log(
        std::clog,
        keywords::auto_flush = true
    )->set_formatter(formatter);
}



void runDongle () try {
    initializeLoggingCore();
    boost::log::sources::logger log;

    boost::asio::io_service ios;
    boost::optional<boost::asio::io_service::work> work {
        boost::in_place(std::ref(ios))
    };

    std::thread ioThread {
        [&] () {
            boost::system::error_code ec;
            auto nHandlers = ios.run(ec);
            BOOST_LOG(log) << "Event loop executed " << nHandlers << " handlers -- " << ec.message();
        }
    };

    Breaker breaker { ios, SIGINT, SIGTERM };
    while (runDongle(ios, breaker)) {
        BOOST_LOG(log) << "runDongle stopped, restarting";
    }
    BOOST_LOG(log) << "runDongle stopped, shutting down";

    work = boost::none;
    ios.stop();
    ioThread.join();
    BOOST_LOG(log) << "IO thread joined, we're outta here";
}
catch (std::exception& e) {
    boost::log::sources::logger log;
    BOOST_LOG(log) << "baromeshd caught exception: " << e.what();
}

