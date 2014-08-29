#ifndef ROBOT_TRANSPORT
#define ROBOT_TRANSPORT

#include "dongleproxy.hpp"

namespace robot {

class Transport {
public:
    Transport (std::string serialId, dongle::Proxy& dongleProxy)
        : mSerialId(serialId)
        , mDongleProxy(dongleProxy) { }

    void sendMessage (const uint8_t* bytes, size_t size) {
        barobo_Dongle_Address destination;
        strcpy((char*)destination.serialId, mSerialId.c_str());
        destination.port = 0;

        barobo_Dongle_Payload payload;
        assert(size <= sizeof(payload.value.bytes));
        memcpy(payload.value.bytes, bytes, size);
        payload.value.size = size;

        auto f = mDongleProxy.fire(rpc::MethodIn<barobo::Dongle>::transmitUnicast {
                destination, payload
        });
        f.get();
    }

private:
    std::string mSerialId;
    dongle::Proxy& mDongleProxy;
};

} // namespace robot

#endif
