#ifndef BAROMESH_DONGLE_PROXY_HPP
#define BAROMESH_DONGLE_PROXY_HPP

#include "dongletransport.hpp"

#include "util/callback.hpp"

#include "rpc/asyncproxy.hpp"
#include "gen-dongle.pb.hpp"

#include <functional>
#include <string>
#include <iostream>

class RobotTransport {};

namespace dongle {

class Proxy : public rpc::AsyncProxy<Proxy, barobo::Dongle> {
public:
    Proxy () {
        mTransport.messageReceived.connect(
            BIND_MEM_CB(&Proxy::deliverMessage, this));
        mTransport.linkUp.connect(
            BIND_MEM_CB(&Proxy::linkUp, this));
        mTransport.linkDown.connect(
            BIND_MEM_CB(&Proxy::linkDown, this));
        mTransport.startReaderThread();
    }

    void linkUp () {
        std::cout << "linkUp called\n";
        mLinked = true;
        for (auto robotTransport : mRobotTransports) {
            //robotTransport.linkUp();
        }
    }

    void linkDown (Transport::DownReason reason) {
        std::cout << "linkDown called " <<
            (reason == Transport::DownReason::NORMALLY ?
            "normally" : "exceptionally") << "\n";
        mLinked = false;
        for (auto robotTransport : mRobotTransports) {
            //robotTransport.linkDown();
        }
    }
    
    void bufferToService (const BufferType& buffer) {
        mTransport.sendMessage(buffer.bytes, buffer.size);
    }

    // A helper function to make a Proxy easier to wire up to an
    // sfp::Context.
    void deliverMessage (const uint8_t* data, size_t size) {
        BufferType buffer;
        // TODO think about what could cause buffer overflows, handle them gracefully
        assert(size <= sizeof(buffer.bytes));
        memcpy(buffer.bytes, data, size);
        buffer.size = size;
        auto status = receiveServiceBuffer(buffer);
        if (rpc::hasError(status)) {
            // TODO shut down gracefully
            printf("Proxy::receiveServiceBuffer returned %s\n", rpc::statusToString(status));
            abort();
        }
    }

    using Attribute = rpc::Attribute<barobo::Dongle>;
    using Broadcast = rpc::Broadcast<barobo::Dongle>;

    void onBroadcast(Attribute::dummyAttribute) { }

    void onBroadcast(Broadcast::receiveUnicast arg) {
        printf("received from %s:%d |",
                arg.source.serialId, arg.source.port);
        if (arg.payload.value.size) {
            for (size_t i = 0; i < arg.payload.value.size; ++i) {
                printf(" %02x", arg.payload.value.bytes[i]);
            }
        }
        else {
            printf(" (empty)");
        }
        printf("\n");
#if 0
        if (arg.source.port == 0) {
            robotMessageReceived(arg.source.serialId, arg.payload.value.bytes, arg.payload.value.size);
        }
        else {
            printf("I dunno what to do with this packet!\n");
        }
#endif
    }

    util::Signal<void(std::string,const uint8_t*,size_t)> robotMessageReceived;

private:
    Transport mTransport;
    RobotTransport mRobotTransports[1];

    std::atomic<bool> mLinked = { false };
};

} // namespace dongle

#endif
