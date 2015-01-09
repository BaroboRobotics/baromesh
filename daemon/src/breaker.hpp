#ifndef BAROMESH_DAEMON_BREAKER_HPP
#define BAROMESH_DAEMON_BREAKER_HPP

#include <boost/asio/io_service.hpp>
#include <boost/asio/signal_set.hpp>

#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>

#include <csignal>

using namespace std::placeholders;

class Breaker {
public:
    Breaker (boost::asio::io_service& ios, int sigNo1, int sigNo2)
        : mSignalSet(ios, sigNo1, sigNo2)
    {}

    class Killswitch {
    public:
        explicit Killswitch (Breaker& b, std::function<void()> handler)
            : mBreaker(b)
        {
            mBreaker.enable(handler);
        }

        ~Killswitch () {
            mBreaker.disable();
        }

    private:
        Breaker& mBreaker;
    };

    friend Killswitch;

private:
    void enable (std::function<void()> handler) {
        std::lock_guard<std::mutex> lock { mMutex };
        assert(!mHandler);
        mHandlerDone = false;
        mHandler = handler;
        mSignalSet.async_wait(std::bind(&Breaker::handleSignal, this, _1, _2));
    }

    void disable () {
        std::unique_lock<std::mutex> lock { mMutex };
        mHandler = nullptr;
        if (!mHandlerDone) {
            mSignalSet.cancel();
            mCondition.wait(lock, [this] () { return mHandlerDone; });
        }
    }

    void handleSignal (boost::system::error_code ec, int sigNo) {
        {
            std::lock_guard<std::mutex> lock { mMutex };
            if (!ec) {
                if (!mHandler) {
                    // signal received in the middle of ~Killswitch, queue it
                    std::raise(sigNo);
                }
                else {
                    mHandler();
                }
            }
            mHandlerDone = true;
        }
        mCondition.notify_one();
    }

    boost::asio::signal_set mSignalSet;

    std::mutex mMutex;
    std::condition_variable mCondition;

    std::function<void()> mHandler;
    bool mHandlerDone;
};

#endif