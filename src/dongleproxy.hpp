#ifndef DONGLEPROXY_HPP_
#define DONGLEPROXY_HPP_

#include <functional>

#include "rpc/asyncproxy.hpp"
#include "gen-dongle.pb.hpp"

class DongleProxy : public rpc::AsyncProxy<DongleProxy, barobo::Dongle> {
public:
    DongleProxy (std::function<void(const BufferType&)> postFunc) :
        mPostFunc(postFunc) {}

    void post (const BufferType& buffer) {
        mPostFunc(buffer);
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
    }

private:
    std::function<void(const BufferType&)> mPostFunc;
};

#endif
