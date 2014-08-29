#ifndef ROBOTPROXY_HPP_
#define ROBOTPROXY_HPP_

#include "robottransport.hpp"

#include "rpc/asyncproxy.hpp"
#include "gen-robot.pb.hpp"

#include <functional>

const char* buttonToString (barobo_Robot_Button button);
const char* buttonStateToString (barobo_Robot_ButtonState state);

class RobotProxy : public rpc::AsyncProxy<RobotProxy, barobo::Robot> {
public:
    RobotProxy (std::string serialId, dongle::Proxy& dongleProxy)
        : mTransport(serialId, dongleProxy) { }

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
    robot::Transport mTransport;
};

#endif
