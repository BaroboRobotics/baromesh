#include "daemonserver.hpp"

#include "baromesh/iocore.hpp"

#include "util/logsafely.hpp"

#include <boost/log/common.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/attributes/timer.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/console.hpp>

#include <exception>
#include <functional>
#include <future>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

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
            << "[T+" << expr::attr<attrs::timer::value_type, util::LogSafely>("Timeline") << "]"
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


// The real main function.
int runDaemon () try {
    auto ioCore = baromesh::IoCore::get(true);

    initializeLoggingCore();
    boost::log::sources::logger log;
    log.add_attribute("Title", boost::log::attributes::constant<std::string>("BAROMESHD"));

    baromesh::DaemonServer daemon { ioCore->ios(), log };
    daemon.run();
    BOOST_LOG(log) << "Shutting down";

    return 0;
}
catch (std::exception& e) {
    boost::log::sources::logger log;
    BOOST_LOG(log) << "baromeshd caught exception: " << e.what();
    return 1;
}