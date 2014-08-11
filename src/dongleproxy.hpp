#ifndef DONGLEPROXY_HPP_
#define DONGLEPROXY_HPP_

#include "util/callback.hpp"

#include "rpc/asyncproxy.hpp"
#include "gen-dongle.pb.hpp"

#include <functional>
#include <string>

class DongleProxy : public rpc::AsyncProxy<DongleProxy, barobo::Dongle> {
public:
    void post (const BufferType& buffer) {
        bufferPosted(buffer.bytes, buffer.size);
    }

    util::Signal<void(const uint8_t*,size_t)> bufferPosted;

    // A helper function to make a DongleProxy easier to wire up to an
    // sfp::Context.
    void deliverMessage (const uint8_t* data, size_t size) {
        BufferType buffer;
        assert(size <= sizeof(buffer.bytes));
        memcpy(buffer.bytes, data, size);
        buffer.size = size;
        auto status = deliver(buffer);
        if (rpc::hasError(status)) {
            // TODO shut down gracefully
            printf("DongleProxy::deliver returned %s\n", rpc::statusToString(status));
            abort();
        }
    }

    using Attribute = rpc::Attribute<barobo::Dongle>;
    using Broadcast = rpc::Broadcast<barobo::Dongle>;

    void onBroadcast(Attribute::dummyAttribute) { }

    void onBroadcast(Broadcast::receiveUnicast arg) {
        printf("received from %.*s:%d |", sizeof(arg.source.serialId),
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

        if (arg.source.port == 0) {
            robotMessageReceived(arg.source.serialId, arg.payload.value.bytes, arg.payload.value.size);
        }
        else {
            printf("I dunno what to do with this packet!\n");
        }
    }

    util::Signal<void(std::string,const uint8_t*,size_t)> robotMessageReceived;
};

#endif
