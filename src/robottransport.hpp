#ifndef ROBOT_TRANSPORT
#define ROBOT_TRANSPORT

#include "dongleproxy.hpp"

namespace robot {

class Transport {
public:
    Transport (std::string serialId) : mSerialId(serialId) {
        mDongleProxy.registerRobotTransport(this);
    }

    ~Transport () {
        mDongleProxy.unregisterRobotTransport(this);
    }

    void sendMessage (const uint8_t* bytes, size_t size) {
        barobo_Dongle_SerialId serialId;
        memcpy(serialId.value, mSerialId.c_str(), 5);

        barobo_Dongle_Payload payload;
        assert(size <= sizeof(payload.value.bytes));
        memcpy(payload.value.bytes, bytes, size);
        payload.value.size = size;

        auto f = mDongleProxy.fire(rpc::MethodIn<barobo::Dongle>::transmitUnicast {
                serialId, payload
        });
        f.get();
    }

    util::Signal<void(const uint8_t*, size_t)> sigMessageReceived;

    std::string serialId() const { return mSerialId; }

private:
    const std::string mSerialId;
    static dongle::Proxy mDongleProxy;
};

} // namespace robot

#endif
