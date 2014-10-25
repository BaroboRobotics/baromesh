#ifndef ROBOTPROXY_HPP_
#define ROBOTPROXY_HPP_

#include "robottransport.hpp"

#include "rpc/asyncproxy.hpp"
#include "gen-robot.pb.hpp"

#include <functional>

namespace robot {

const char* buttonToString (barobo_Robot_Button button);
const char* buttonStateToString (barobo_Robot_ButtonState state);

class Proxy : public rpc::AsyncProxy<Proxy, barobo::Robot> {
public:
    Proxy (std::string serialId) : mTransport(serialId) {
        mTransport.sigMessageReceived.connect(
            BIND_MEM_CB(&Proxy::deliverMessage, this));
    }

    void bufferToService (const BufferType& buffer) {
        mTransport.sendMessage(buffer.bytes, buffer.size);
    }

    using Broadcast = rpc::Broadcast<barobo::Robot>;

    void onBroadcast (Broadcast::buttonEvent in) {
        buttonEvent(in.button, in.state, in.timestamp);
    }

    void onBroadcast (Broadcast::encoderEvent in) {
        encoderEvent(in.encoder, in.value, in.timestamp);
    }

    void onBroadcast (Broadcast::jointEvent in) {
        jointEvent(in.joint, in.event, in.timestamp);
    }

    void onBroadcast (Broadcast::accelerometerEvent in) {
        accelerometerEvent(in.x, in.y, in.z, in.timestamp);
    }

    util::Signal<void(int,int, int)> buttonEvent;
    util::Signal<void(double,double,double,int)> accelerometerEvent;
    util::Signal<void(int,double, int)> encoderEvent;
    util::Signal<void(int,int,int)> jointEvent;

private:
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
            printf("Robot::receiveServiceBuffer returned %s\n", rpc::statusToString(status));
        }
    }

    Transport mTransport;
};

} // namespace robot
#endif
