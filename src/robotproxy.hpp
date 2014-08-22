#ifndef ROBOTPROXY_HPP_
#define ROBOTPROXY_HPP_

#include <functional>

#include "rpc/asyncproxy.hpp"
#include "gen-robot.pb.hpp"

class RobotProxy : public rpc::AsyncProxy<RobotProxy, barobo::Robot> {
public:
    RobotProxy (std::function<void(const BufferType&)> postFunc) :
        mPostFunc(postFunc) {}

    void post (const BufferType& buffer) {
        mPostFunc(buffer);
    }

    using Attribute = rpc::Attribute<barobo::Robot>;
    using Broadcast = rpc::Broadcast<barobo::Robot>;

    void onBroadcast(Attribute::buzzerFrequency attr) {
        std::cout << "Received buzzerFrequency broadcast. " << attr.value << "\n";
    }

    void onBroadcast(Broadcast::buttonEvent in) {
        auto buttonToString = [] (barobo_Robot_Button button) {
            switch (button) {
                case barobo_Robot_Button_POWER:
                    return "POWER";
                case barobo_Robot_Button_A:
                    return "A";
                case barobo_Robot_Button_B:
                    return "B";
                default:
                    return "(unknown)";
            }
        };
        auto buttonStateToString = [] (barobo_Robot_ButtonState state) {
            switch (state) {
                case barobo_Robot_ButtonState_UP:
                    return "UP";
                case barobo_Robot_ButtonState_DOWN:
                    return "DOWN";
                default:
                    return "(unknown)";
            }
        };
        std::cout << "Received button event: timestamp(" << in.timestamp
                  << ") button(" << buttonToString(in.button)
                  << ") state(" << buttonStateToString(in.state)
                  << ")\n";
    }

private:
    std::function<void(const BufferType&)> mPostFunc;
};

#endif
