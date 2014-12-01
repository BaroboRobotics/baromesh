#ifndef BAROMESH_DONGLE_PROXY_HPP
#define BAROMESH_DONGLE_PROXY_HPP

#include <boost/log/common.hpp>
#include <boost/log/sources/logger.hpp>

#include "dongletransport.hpp"

#include "util/callback.hpp"

#include "rpc/asyncproxy.hpp"
#include "gen-dongle.pb.hpp"

#include <functional>
#include <string>
#include <iostream>

namespace robot {
    class Transport;
}

namespace dongle {

class Proxy : public rpc::AsyncProxy<Proxy, barobo::Dongle> {
public:
    Proxy () {
        std::cout << "Proxy Cons." << std::endl;
        mTransport.sigMessageReceived.connect(
            BIND_MEM_CB(&Proxy::deliverMessage, this));
        mTransport.sigDisconnected.connect(
            BIND_MEM_CB(&Proxy::disconnected, this));
        mTransport.sigConnecting.connect(
            BIND_MEM_CB(&Proxy::connecting, this));
        mTransport.sigConnected.connect(
            BIND_MEM_CB(&Proxy::connected, this));
        std::cout << "Start reader thread..." << std::endl;
        mTransport.startReaderThread();
        std::cout << "Proxy Cons done." << std::endl;
    }

    void disconnected () {
        std::cout << "RECEIVED disconnected" << "\n";
        mLinked = false;
        for (auto robotTransport : mRobotTransports) {
            //robotTransport.linkDown();
        }
    }

    void connecting() {
        std::cout << "RECEIVED connecting" << "\n";
    }

    void connected () {
        std::cout << "RECEIVED connected\n";
        mLinked = true;
        for (auto robotTransport : mRobotTransports) {
            //robotTransport.linkUp();
        }
    }

    void bufferToService (const BufferType& buffer) {
        BOOST_LOG_NAMED_SCOPE("dongle::Proxy::bufferToService");
        BOOST_LOG(mLog) << "sending message through dongle::Transport";
        mTransport.sendMessage(buffer.bytes, buffer.size);
        BOOST_LOG(mLog) << "message sent";
    }

    // A helper function to make a Proxy easier to wire up to a transport
    void deliverMessage (const uint8_t* data, size_t size) {
        BOOST_LOG_NAMED_SCOPE("dongle::Proxy::deliverMessage");
        BufferType buffer;
        // TODO think about what could cause buffer overflows, handle them gracefully
        assert(size <= sizeof(buffer.bytes));
        memcpy(buffer.bytes, data, size);
        buffer.size = size;
        BOOST_LOG(mLog) << "receiving service buffer";
        auto status = receiveServiceBuffer(buffer);
        if (rpc::hasError(status)) {
            // TODO shut down gracefully?
            BOOST_LOG(mLog) << "receiveServiceBuffer returned " << rpc::statusToString(status);
        }
    }

    using Broadcast = rpc::Broadcast<barobo::Dongle>;

    void onBroadcast(Broadcast::receiveUnicast arg);

    bool registerRobotTransport(robot::Transport* rt);
    bool unregisterRobotTransport(robot::Transport* rt);

private:
    boost::log::sources::logger_mt mLog;

    Transport mTransport;
    std::map<std::string, robot::Transport*> mRobotTransports;

    std::atomic<bool> mLinked = { false };
};

} // namespace dongle

#endif
