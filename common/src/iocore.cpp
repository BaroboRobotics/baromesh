#include "baromesh/iocore.hpp"

#include <boost/utility/in_place_factory.hpp>

#include <boost/log/sources/record_ostream.hpp>

#include <exception>
#include <mutex>
#include <string>

#include <cstdlib>

namespace baromesh {

IoCore::IoCore ()
    : mWork(boost::in_place(std::ref(mIos)))
    // Run the io_service in a separate thread
    , mNHandlers(std::async(std::launch::async, [this] () { return mIos.run(); }))
{
}

std::shared_ptr<IoCore> IoCore::get () {
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock {mutex};

    static auto wp = std::weak_ptr<IoCore>{};
    auto p = wp.lock();
    if (!p) {
        p = std::shared_ptr<IoCore>(new IoCore{});
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