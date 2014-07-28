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

    void onBroadcast(Attribute::ledColor color) {
        std::cout << "Received ledColor broadcast. " << color.value << "\n";
    }

    void onBroadcast(Broadcast::dummyBroadcast) {
    }

private:
    std::function<void(const BufferType&)> mPostFunc;
};

#endif
