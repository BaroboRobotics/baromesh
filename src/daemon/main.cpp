#include "daemonserver.hpp"
#include "dongledevicepath.hpp"

#include "sfp/asio/messagequeue.hpp"

#include "rpc/message.hpp"
#include "rpc/version.hpp"
#include "rpc/asio/client.hpp"
#include "rpc/asio/server.hpp"
#include "rpc/asio/tcppolyserver.hpp"

#include "util/hexdump.hpp"
//#include "util/log.hpp"
#include "util/monospawn.hpp"

#include <boost/asio.hpp>
#include <boost/asio/spawn.hpp>

#include <boost/log/common.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/attributes/timer.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/console.hpp>

#include <functional>
#include <iostream>
#include <string>

using namespace std::placeholders;

using Tcp = boost::asio::ip::tcp;
using TcpMessageQueue = sfp::asio::MessageQueue<Tcp::socket>;

static const int kBaudRate = 230400;

void dongleCoroutine (boost::asio::io_service& ios, boost::asio::yield_context yield) {
    boost::log::sources::logger log;
    log.add_attribute("Title", boost::log::attributes::constant<std::string>("DONGLECORO"));

    try {
        BOOST_LOG(log) << "Waiting for dongle";
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


        boost::log::sources::logger dongleClLog;
        dongleClLog.add_attribute("Title", boost::log::attributes::constant<std::string>("DONGLE-CL"));
        baromesh::Dongle dongle { ios, dongleClLog };

        auto& stream = dongle.client().messageQueue().stream();
        stream.open(devicePath);
        stream.set_option(boost::asio::serial_port_base::baud_rate(kBaudRate));

        dongle.client().messageQueue().asyncHandshake(yield);
        asyncConnect(dongle.client(), std::chrono::milliseconds(100), yield);

        Tcp::resolver resolver { ios };
        auto iter = resolver.async_resolve(std::string("42000"), yield);
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
        nullMq.setRoute(dongle, "....");

        uint8_t buf[1];
        nullMq.asyncReceive(boost::asio::buffer(buf),
            [server, log] (boost::system::error_code ec, size_t) mutable {
                if (!ec) {
                    BOOST_LOG(log) << "Actually read something from serial ID \"....\" . "
                                   << "That should be impossible.";
                }
                else {
                    BOOST_LOG(log) << "Dongle error: " << ec.message();
                }
                server->cancel(ec);
            });

        rpc::asio::TcpPolyServer::RequestId requestId;
        barobo_rpc_Request request;

        while (true) {
            BOOST_LOG(daemonSvLog) << "awaiting connection";
            baromesh::DaemonServer daemon { *server, dongle, daemonSvLog };
            asyncRunServer(*server, daemon, yield);
            BOOST_LOG(daemonSvLog) << "disconnected";
        }
    }
    catch (boost::system::system_error& e) {
        BOOST_LOG(log) << "Error in dongle coroutine: " << e.what();
    }
    BOOST_LOG(log) << "Restarting dongle coroutine";
    boost::asio::spawn(ios, std::bind(&dongleCoroutine, std::ref(ios), _1));
}

static void initializeLoggingCore () {
    namespace expr = boost::log::expressions;
    namespace keywords = boost::log::keywords;
    namespace attrs = boost::log::attributes;

    boost::log::add_common_attributes();

    auto core = boost::log::core::get();
    core->add_global_attribute("Timeline", attrs::timer());
    core->add_global_attribute("Scope", attrs::named_scope());

    boost::log::formatter formatter =
        expr::stream
            << "[T+" << expr::attr<attrs::timer::value_type>("Timeline") << "]"
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


int main (int argc, char** argv) try {
    util::Monospawn sentinel { "baromeshd", std::chrono::seconds(1) };

    initializeLoggingCore();
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