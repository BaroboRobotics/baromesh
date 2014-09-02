#ifndef BAROMESH_DONGLE_PROXY_HPP
#define BAROMESH_DONGLE_PROXY_HPP

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
        mTransport.sigMessageReceived.connect(
            BIND_MEM_CB(&Proxy::deliverMessage, this));
        mTransport.sigDisconnected.connect(
            BIND_MEM_CB(&Proxy::disconnected, this));
        mTransport.sigConnecting.connect(
            BIND_MEM_CB(&Proxy::connecting, this));
        mTransport.sigConnected.connect(
            BIND_MEM_CB(&Proxy::connected, this));
        mTransport.startReaderThread();
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
        mTransport.sendMessage(buffer.bytes, buffer.size);
    }

    // A helper function to make a Proxy easier to wire up to a transport
    void deliverMessage (const uint8_t* data, size_t size) {
        BufferType buffer;
        // TODO think about what could cause buffer overflows, handle them gracefully
        assert(size <= sizeof(buffer.bytes));
        memcpy(buffer.bytes, data, size);
        buffer.size = size;
        auto status = receiveServiceBuffer(buffer);
        if (rpc::hasError(status)) {
            // TODO shut down gracefully?
            printf("Dongle::receiveServiceBuffer returned %s\n", rpc::statusToString(status));
        }
    }

    using Attribute = rpc::Attribute<barobo::Dongle>;
    using Broadcast = rpc::Broadcast<barobo::Dongle>;

    void onBroadcast(Attribute::dummyAttribute) { }
    void onBroadcast(Broadcast::receiveUnicast arg);

    bool registerRobotTransport(robot::Transport* rt);
    bool unregisterRobotTransport(robot::Transport* rt);

private:
    Transport mTransport;
    std::map<std::string, robot::Transport*> mRobotTransports;

    std::atomic<bool> mLinked = { false };
};

} // namespace dongle

#endif
