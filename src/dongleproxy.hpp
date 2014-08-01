#ifndef DONGLEPROXY_HPP_
#define DONGLEPROXY_HPP_

#include <functional>
#include <string>

#include "rpc/asyncproxy.hpp"
#include "gen-dongle.pb.hpp"

class DongleProxy : public rpc::AsyncProxy<DongleProxy, barobo::Dongle> {
public:
    DongleProxy (std::function<void(const BufferType&)> postFunc,
            std::function<void(std::string,const uint8_t*,size_t)> receiveFunc)
        : mPostFunc(postFunc)
        , mReceiveFunc(receiveFunc) { }

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

        if (arg.source.port == 0) {
            mReceiveFunc(arg.source.serialId, arg.payload.value.bytes, arg.payload.value.size);
        }
        else {
            printf("I dunno what to do with this packet!\n");
        }
    }

private:
    std::function<void(const BufferType&)> mPostFunc;
    std::function<void(std::string,const uint8_t*,size_t)> mReceiveFunc;
};

#endif
