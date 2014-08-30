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

    using Attribute = rpc::Attribute<barobo::Robot>;
    using Broadcast = rpc::Broadcast<barobo::Robot>;

    void onBroadcast(Attribute::buzzerFrequency attr) {
        std::cout << "Received buzzerFrequency broadcast. " << attr.value << "\n";
    }

    void onBroadcast(Broadcast::buttonEvent in) {
        std::cout << "Received button event: timestamp(" << in.timestamp
                  << ") button(" << buttonToString(in.button)
                  << ") state(" << buttonStateToString(in.state)
                  << ")\n";
    }

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
            // TODO shut down gracefully
            printf("Robot::receiveServiceBuffer returned %s\n", rpc::statusToString(status));
            abort();
        }
    }

    Transport mTransport;
};

} // namespace robot
#endif
