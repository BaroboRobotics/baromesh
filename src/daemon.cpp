#include "daemon.hpp"
#include "iocore.hpp"

#include <chrono>
#include <memory>
#include <mutex>

#include <cstdlib>

namespace baromesh {


// Get a shared pointer to the daemon singleton. If the singleton does not
// already exist, create it. If all shared_ptrs returned from this function
// are destroyed, the daemon singleton will be destroyed.
std::shared_ptr<Daemon> daemonInstance () {
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock { mutex };

    bool enable = false;
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
    boost::log::core::get()->set_logging_enabled(enable);

    boost::log::sources::logger log;
    log.add_attribute("Title", boost::log::attributes::constant<std::string>("DAEMON"));

    static std::weak_ptr<Daemon> d;

    auto p = d.lock();
    if (!p) {
        BOOST_LOG(log) << "Creating daemon singleton";
        // TODO, spawn daemon process on demand
        p = std::make_shared<Daemon>(ioCore().ios(), log);
        d = p;
    }
    BOOST_LOG(log) << p.use_count() << " daemon pointers exist";
    return p;
}

void asyncAcquireDaemonImpl (AcquireDaemonHandler handler) {
    const std::chrono::milliseconds kDaemonConnectTimeout { 200 };
    const std::chrono::milliseconds kDaemonKeepaliveTimeout { 500 };

    static boost::asio::io_service::strand strand { ioCore().ios() };
    static std::queue<AcquireDaemonHandler> handlerQueue;
    static std::atomic<bool> daemonIsAcquired = { false };

    auto d = daemonInstance();
    if (!d->client().messageQueue().stream().is_open()) {
        daemonIsAcquired = false;
    }

    if (daemonIsAcquired) {
        ioCore().ios().post(std::bind(handler, boost::system::error_code(), d));
        return;
    }

    // Capturing a static variable by reference causes a compiler warning about
    // capture of a variable with non-automatic storage duration. I'm not sure
    // that this particular case is actually a problem, but work around the
    // warning anyway.
    auto handlers = &handlerQueue;
    auto acquired = &daemonIsAcquired;

    auto postAcquires = [=] (boost::system::error_code ec, std::shared_ptr<Daemon> d) {
        while (handlers->size()) {
            auto h = handlers->front();
            handlers->pop();
            ioCore().ios().post(std::bind(h, ec, d));
        }
    };

    boost::log::sources::logger log;
    BOOST_LOG(log) << "Acquiring daemon ...";
    strand.post([=] () mutable {
        handlers->push(handler);
        if (1 == handlers->size()) {
            using Tcp = boost::asio::ip::tcp;
            // TODO: what to do about gracefully disconnecting? Do we care?
            auto resolver = std::make_shared<Tcp::resolver>(ioCore().ios());
            resolver->async_resolve(Tcp::resolver::query("127.0.0.1", "42000"),
            [=] (boost::system::error_code ec, Tcp::resolver::iterator iter) mutable {
                (void)resolver; // reference it so its lifetime is extended
                if (!ec) {
                    BOOST_LOG(log) << "Resolved daemon to " << iter->endpoint();
                    boost::asio::async_connect(d->client().messageQueue().stream(), iter,
                    [=] (boost::system::error_code ec, Tcp::resolver::iterator iter) mutable {
                        if (!ec) {
                            BOOST_LOG(log) << "Connected to daemon at " << iter->endpoint();
                            d->client().messageQueue().asyncHandshake(
                            [=] (boost::system::error_code ec) mutable {
                                if (!ec) {
                                    BOOST_LOG(log) << "Shook hands with the daemon";
                                    asyncConnect(d->client(), kDaemonConnectTimeout,
                                    [=] (boost::system::error_code ec, rpc::ServiceInfo info) mutable {
                                        if (!ec) {
                                            *acquired = true;
#warning check for version mismatch
                                            BOOST_LOG(log) << "Daemon has RPC version " << info.rpcVersion()
                                                           << ", interface version " << info.interfaceVersion();
                                            postAcquires(boost::system::error_code(), d);
                                        }
                                        else {
                                            if (ec != boost::asio::error::operation_aborted) {
                                                d->client().close();
                                            }
                                            BOOST_LOG(log) << "Daemon RPC connection failed: " << ec.message();
                                            postAcquires(ec, nullptr);
                                        }
                                    });
                                }
                                else {
                                    if (ec != boost::asio::error::operation_aborted) {
                                        d->client().close();
                                    }
                                    BOOST_LOG(log) << "Daemon handshake failed: " << ec.message();
                                    postAcquires(ec, nullptr);
                                }
                            });
                        }
                        else {
                            if (ec != boost::asio::error::operation_aborted) {
                                d->client().close();
                            }
                            BOOST_LOG(log) << "Daemon TCP connection failed: " << ec.message();
                            postAcquires(ec, nullptr);
                        }
                    });
                }
                else {
                    BOOST_LOG(log) << "Daemon endpoint resolution failed: " << ec.message();
                    postAcquires(ec, nullptr);
                }
            });
        }
    });
}

} // namespace baromesh
