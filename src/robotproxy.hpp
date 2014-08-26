#ifndef ROBOTPROXY_HPP_
#define ROBOTPROXY_HPP_

#include <functional>

#include "rpc/asyncproxy.hpp"
#include "gen-robot.pb.hpp"

const char* buttonToString (barobo_Robot_Button button);
const char* buttonStateToString (barobo_Robot_ButtonState state);

class RobotProxy : public rpc::AsyncProxy<RobotProxy, barobo::Robot> {
public:
    RobotProxy (std::function<void(const BufferType&)> postFunc) :
        mPostFunc(postFunc) {}

    void bufferToService (const BufferType& buffer) {
        mPostFunc(buffer);
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
    std::function<void(const BufferType&)> mPostFunc;
};

#endif
