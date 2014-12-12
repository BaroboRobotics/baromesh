#include "iocore.hpp"

#include <mutex>

namespace baromesh {

IoCore::IoCore ()
    : mWork(boost::in_place(std::ref(mIos)))
{}

void IoCore::init () {
    std::thread t {
        static_cast<size_t(boost::asio::io_service::*)()>(&boost::asio::io_service::run),
        &mIos
    };
    mThread.swap(t);
}

IoCore::~IoCore () {
    mWork = boost::none;
    //mIos.stop();
    if (mThread.joinable()) {
        mThread.join();
    }
}

IoCore& ioCore () {
    static IoCore core;
    // Use a post-constructor init routine with call_once to start the core's
    // internal thread on the first call to ioCore. Hopefully this will allow
    // Python to use the library easier.
    static std::once_flag flag;
    std::call_once(flag, &IoCore::init, &core);
    return core;
}

} // namespace baromesh