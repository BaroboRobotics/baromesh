#include "iocore.hpp"

#include <boost/log/sources/record_ostream.hpp>

#include <exception>
#include <mutex>
#include <string>

#include <cstdlib>

namespace baromesh {

IoCore::IoCore (boost::optional<bool> enable)
    : mLoggingCore(boost::log::core::get())
    , mWork(boost::in_place(std::ref(mIos)))
    // Run the io_service in a separate thread
    , mNHandlers(std::async(std::launch::async, [this] () { return mIos.run(); }))
{
    // The environment variable overrides whatever we were passed.
    if (const auto enableStr = std::getenv("BAROMESH_LOG_ENABLE")) {
        try {
            enable = std::stoi(enableStr);
        }
        catch (std::exception& e) {
            boost::log::sources::logger log;
            BOOST_LOG(log) << "BAROMESH_LOG_ENABLE environment variable is set,"
                           << " but not a numeric value: " << e.what();
        }
    }

    if (enable) {
        mLoggingCore->set_logging_enabled(*enable);
    }
}

std::shared_ptr<IoCore> IoCore::get (boost::optional<bool> enableLogging) {
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock {mutex};

    static auto wp = std::weak_ptr<IoCore>{};
    auto p = wp.lock();
    if (!p) {
        p = std::shared_ptr<IoCore>(new IoCore{enableLogging});
        wp = p;
    }
    return p;
}

IoCore::~IoCore () {
    mWork = boost::none;
    try {
        // When mNHandlers is ready, the io_service::run thread must be all
        // cleaned up, and it's safe to continue destruction.
        auto nHandlers = mNHandlers.get();
        BOOST_LOG(mLog) << "ran " << nHandlers << " handlers to completion";
    }
    catch (std::exception& e) {
        BOOST_LOG(mLog) << "ran into " << e.what();
    }
}

} // namespace baromesh