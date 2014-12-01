#include "daemon.hpp"
#include "iocore.hpp"

#include <boost/asio/spawn.hpp>

#include <chrono>
#include <memory>
#include <mutex>

namespace baromesh {

namespace {

static const std::chrono::milliseconds kDaemonConnectTimeout { 200 };

}

// Get a shared pointer to the daemon singleton. If the singleton does not
// already exist, create it. If all shared_ptrs returned from this function
// are destroyed, the daemon singleton will be destroyed.
std::shared_ptr<Daemon> daemonInstance () {
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock { mutex };

    boost::log::sources::logger log;
    log.add_attribute("Title", boost::log::attributes::constant<std::string>("DAEMON"));

    static std::weak_ptr<Daemon> d;

    auto p = d.lock();
    if (!p) {
        BOOST_LOG(log) << "Creating daemon singleton";
        // TODO, spawn daemon coroutine
        p = std::make_shared<Daemon>(ioCore().ios(), log);
        d = p;
    }
    BOOST_LOG(log) << p.use_count() << " daemon pointers exist";
    return p;
}

void asyncAcquireDaemonImpl (AcquireDaemonHandler handler) {
    static boost::asio::io_service::strand strand { ioCore().ios() };
    static std::queue<AcquireDaemonHandler> handlerQueue;

    // Capturing a static variable by reference causes a compiler warning about
    // capture of a variable with non-automatic storage duration. I'm not sure
    // that this particular case is actually a problem, but work around the
    // warning anyway.
    auto handlers = &handlerQueue;

    auto postAcquires = [=] (boost::system::error_code ec, std::shared_ptr<Daemon> d) {
        while (handlers->size()) {
            auto h = handlers->front();
            handlers->pop();
            ioCore().ios().post(std::bind(h, ec, d));
        }
    };

    strand.post([=] () {
        handlers->push(handler);
        if (1 == handlers->size()) {
            auto d = daemonInstance();
            boost::asio::spawn(strand,
                [=] (boost::asio::yield_context yield) {
                    boost::log::sources::logger log;
                    try {
                        // TODO: what to do about gracefully disconnecting? Do we care?
                        boost::asio::ip::tcp::resolver resolver { ioCore().ios() };
                        auto iter = resolver.async_resolve(decltype(resolver)::query("localhost", "42000"), yield);
                        boost::asio::async_connect(d->client().messageQueue().stream(), iter, yield);
                        d->client().messageQueue().asyncHandshake(yield);
                        asyncConnect(d->client(), kDaemonConnectTimeout, yield);
                        BOOST_LOG(log) << "Connected to daemon";
                        postAcquires(boost::system::error_code(), d);
                    }
                    catch (boost::system::system_error& e) {
                        BOOST_LOG(log) << "Error connecting to daemon: " << e.what();
                        postAcquires(e.code(), nullptr);
                    }
                });
        }
    });
}

} // namespace baromesh