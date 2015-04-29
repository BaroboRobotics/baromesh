#include "daemonserver.hpp"

#include "baromesh/iocore.hpp"
#include "baromesh/log.hpp"

#include <boost/program_options/parsers.hpp>

#include <boost/predef.h>

#include <exception>
#include <functional>
#include <future>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

namespace po = boost::program_options;

// The real main function.
int runDaemon (int argc, char** argv) try {
    auto optsDesc = po::options_description{"Linkbot Labs Service command line options"};
    optsDesc.add_options()
        ("help", "display help information")
    ;

    auto defaultLogFile = std::string{"baromeshd.log"};
    if (BOOST_OS_UNIX) {
        defaultLogFile.insert(0, "/var/log/");
    }

    optsDesc.add(baromesh::log::optionsDescription(defaultLogFile));

    auto options = boost::program_options::variables_map{};
    po::store(po::parse_command_line(argc, argv, optsDesc), options);
    po::notify(options);

    if (options.count("help")) {
        std::cout << optsDesc << std::endl;
        return 0;
    }

    baromesh::log::initialize("baromeshd", options);

    boost::log::sources::logger log;
    log.add_attribute("Title", boost::log::attributes::constant<std::string>("BAROMESHD"));

    auto ioCore = baromesh::IoCore::get();
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

namespace baromesh {
uint32_t computerId () {
    static std::random_device rd{};
    static auto gen = std::mt19937{rd()};
    static auto dis = std::uniform_int_distribution<uint32_t>{};
    static auto id = dis(gen);
    return id;
}
}

