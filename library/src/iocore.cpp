#include "iocore.hpp"

#include <boost/log/sources/record_ostream.hpp>

namespace baromesh {

IoCore::IoCore (boost::optional<bool> enableLogging)
    : mLoggingCore(boost::log::core::get())
    , mWork(boost::in_place(std::ref(mIos)))
{
    maybeEnableLogging(enableLogging);
    startThread();
}

void IoCore::maybeEnableLogging (boost::optional<bool> enable) {
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

void IoCore::startThread () {
    auto p = std::make_shared<int>(0);
    mToken = p;
    std::thread t {
        [this, p] () mutable {
            try {
                auto nHandlers = mIos.run();
                BOOST_LOG(mLog) << "ran " << nHandlers << " handlers to completion";
            }
            catch (std::exception& e) {
                BOOST_LOG(mLog) << "ran into " << e.what();
            }
            p.reset();
        }
    };
    t.swap(mThread);
}

std::shared_ptr<IoCore> IoCore::get (boost::optional<bool> enableLogging) {
    //static auto core = std::shared_ptr<IoCore>(new IoCore{enableLogging});
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
    while (!mToken.expired()) {
        std::this_thread::yield();
    }
#if 0
    try {
        auto nHandlers = mNHandlers.get();
        BOOST_LOG(mLog) << "ran " << nHandlers << " handlers to completion";
    }
    catch (std::exception& e) {
        BOOST_LOG(mLog) << "ran into " << e.what();
    }
#endif
    mThread.detach();
#if 0
    if (mThread.joinable()) {
        mThread.join();
    }
#endif
}

} // namespace baromesh